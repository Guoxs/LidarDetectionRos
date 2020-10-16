/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-10 16:49:11
 */

#include <map>
#include <numeric>

#include "hm_tracker.h"
#include "../common/geometry_util.h"
#include "../feature/feature_descriptor.h"
#include "hungarian_matcher.h"
#include "kalman_filter.h"
#include "../tracked_obj/track_object_distance.h"


bool HmObjectTracker::Init(const ros::NodeHandle& nh)
{
  // Initialize tracker's configs
  nh_=nh;
  ros::param::get("~track_cached_history_size_maximum",track_cached_history_size_maximum);
  ros::param::get("~track_consecutive_invisible_maximum",track_consecutive_invisible_maximum);
  ros::param::get("~track_visible_ratio_minimum",track_visible_ratio_minimum);
  ros::param::get("~collect_age_minimum",collect_age_minimum);
  ros::param::get("~collect_consecutive_invisible_maximum",collect_consecutive_invisible_maximum);
  ros::param::get("~acceleration_noise_maximum",acceleration_noise_maximum);
  ros::param::get("~speed_noise_maximum",speed_noise_maximum);
  ros::param::get("~match_distance_maximum",match_distance_maximum);
  ros::param::get("~location_distance_weight",location_distance_weight);
  ros::param::get("~direction_distance_weight",direction_distance_weight);
  ros::param::get("~bbox_size_distance_weight",bbox_size_distance_weight);
  ros::param::get("~point_num_distance_weight",point_num_distance_weight);
  ros::param::get("~histogram_distance_weight",histogram_distance_weight);
  ros::param::get("~histogram_bin_size",histogram_bin_size);
  ros::param::get("~use_adaptive",use_adaptive);
  ros::param::get("~measurement_noise",measurement_noise);
  ros::param::get("~initial_velocity_noise",initial_velocity_noise);
  ros::param::get("~xy_propagation_noise",xy_propagation_noise);
  ros::param::get("~z_propagation_noise",z_propagation_noise);
  ros::param::get("~breakdown_threshold_maximum",breakdown_threshold_maximum);


  // A. Basic tracker setup
  // load match method
  matcher_.reset(new HungarianMatcher());

  // load filter method
  // load track cached history size maximum
  if (!ObjectTrack::SetTrackCachedHistorySizeMaximum(this->track_cached_history_size_maximum)) 
  {
    cout << "Failed to set track cached history size maximum! " << name()<<endl;
    return false;
  }
  // load track consevutive invisible maximum
  if (!ObjectTrackSet::SetTrackConsecutiveInvisibleMaximum(this->track_consecutive_invisible_maximum)) 
  {
    cout << "Failed to set track consecutive invisible maximum! " << name() <<endl;
    return false;
  }
  // load track visible ratio minimum
  if (!ObjectTrackSet::SetTrackVisibleRatioMinimum(this->track_visible_ratio_minimum)) 
  {
    cout << "Failed to set track visible ratio minimum! " << name()<<endl;
    return false;
  }
  // check collect age minimum
  if (this->collect_age_minimum < 0) 
  {
    cout << "invalid collect age minimum of " << name()<<endl;
    return false;
  }

  // check collect consecutive invisible maximum
  if (this->collect_consecutive_invisible_maximum < 0) 
  {
    cout << "invalid collect consecutive invisible maximum of " << name()<<endl;
    return false;
  }

  // load acceleration maximum
  if (!ObjectTrack::SetAccelerationNoiseMaximum(this->acceleration_noise_maximum)) 
  {
    cout << "Failed to set acceleration noise maximum! " << name()<<endl;
    return false;
  }
  // load speed noise maximum
  if (!ObjectTrack::SetSpeedNoiseMaximum(this->speed_noise_maximum)) 
  {
    cout << "Failed to set speed noise maximum! " << name()<<endl;
    return false;
  }

  // B. Matcher setup
  // load match distance maximum
  
  if (!HungarianMatcher::SetMatchDistanceMaximum(this->match_distance_maximum)) 
  {
    cout << "Failed to set match distance maximum! " << name()<<endl;
    return false;
  }
  
  // load location distance weight
  if (!TrackObjectDistance::SetLocationDistanceWeight(this->location_distance_weight)) 
  {
    cout << "Failed to set location distance weight! " << name()<<endl;
    return false;
  }
  // load direction distance weight
  if (!TrackObjectDistance::SetDirectionDistanceWeight(this->direction_distance_weight)) 
  {
    cout << "Failed to set direction distance weight! " << name()<<endl;
    return false;
  }
  // load bbox size distance weight
  if (!TrackObjectDistance::SetBboxSizeDistanceWeight(this->bbox_size_distance_weight)) 
  {
    cout << "Failed to set bbox size distance weight! " << name()<<endl;
    return false;
  }
  // load point num distance weight
  if (!TrackObjectDistance::SetPointNumDistanceWeight(this->point_num_distance_weight)) 
  {
    cout << "Failed to set point num distance weight! " << name()<<endl;
    return false;
  }
  // load histogram distance weight
  if (!TrackObjectDistance::SetHistogramDistanceWeight(this->histogram_distance_weight)) 
  {
    cout << "Failed to set histogram distance weight! " << name()<<endl;
    return false;
  }
  use_histogram_for_match_ = this->histogram_distance_weight > FLT_EPSILON;
  if (this->histogram_bin_size <= 0) 
  {
    cout << "invalid histogram bin size of " << name()<<endl;
    return false;
  }

  // C. Filter setup
  
  double association_score_maximum = this->match_distance_maximum;
  KalmanFilter::SetUseAdaptive(this->use_adaptive);
  if (!KalmanFilter::SetAssociationScoreMaximum(association_score_maximum)) 
  {
    cout << "Failed to set association score maximum! " << name();
    return false;
  }
  if (!KalmanFilter::InitParams(
      this->measurement_noise, this->initial_velocity_noise,
      this->xy_propagation_noise,this->z_propagation_noise)) 
  {
    cout << "Failed to set params for kalman filter! " << name();
    return false;
  }
  if (!KalmanFilter::SetBreakdownThresholdMaximum(
      this->breakdown_threshold_maximum)) 
  {
    cout << "Failed to set breakdown threshold maximum! " << name();
    return false;
  }
  
  return true;
}

const std::vector<ObjectTrackPtr>& HmObjectTracker::GetObjectTracks() const 
{
  return object_tracks_.GetTracks();
}

bool HmObjectTracker::Track(
    const std::vector<std::shared_ptr<Object>>& objects, double timestamp,
    const Eigen::Matrix4f& trans,
    std::vector<std::shared_ptr<Object>>* tracked_objects) 
  {
  // A. track setup
  if (tracked_objects == nullptr) 
  {
    cout<<"tracked_objects is nullptr"<<endl;
    return false;
  }
  if (!valid_) 
  {
    valid_ = true;
    return InitializeTrack(objects, timestamp, trans, tracked_objects);
  }
  Eigen::Matrix4f velo2world_pose = Eigen::Matrix4f::Identity();
  // if (options.velodyne_trans != nullptr) 
  // {
  //   velo2world_pose = *(options.velodyne_trans);
  // } 
  // else 
  // {
  //   cout << "Input velodyne_trans is null";
  //   return false;
  // }
  velo2world_pose=trans;
  double time_diff = timestamp - time_stamp_;
  time_stamp_ = timestamp;

  // B. preprocessing
  // B.1 transform given pose to local one
  TransformPoseGlobal2Local(&velo2world_pose);
  // cout << "velo2local_pose\n" << velo2world_pose<<endl;
  // B.2 construct objects for tracking
  std::vector<std::shared_ptr<TrackedObject>> transformed_objects;
  ConstructTrackedObjects(objects, &transformed_objects, velo2world_pose,
                         trans);

  // C. prediction
  std::vector<Eigen::VectorXf> tracks_predict;
  ComputeTracksPredict(&tracks_predict, time_diff);

  // D. match objects to tracks
  std::vector<std::pair<int, int>> assignments;
  std::vector<int> unassigned_objects;
  std::vector<int> unassigned_tracks;
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.GetTracks();
  if (matcher_ != nullptr) 
  {
    matcher_->Match(&transformed_objects, tracks, tracks_predict, &assignments,
                    &unassigned_tracks, &unassigned_objects);
  } 
  else 
  {
    cout << "matcher_ is not initiated. Please call Init() function before "
              "other functions.";
    return false;
  }
  // cout << "之前跟踪个数为： " << tracks.size() << " 当前目标个数为： "
  //        <<transformed_objects.size() << " 匹配的目标的个数为： " 
  //        << assignments.size()  << " 未匹配的目标的个数为： "
  //        << unassigned_objects.size() << " 时间差为： " << time_diff<<endl;

  // E. update tracks
  // E.1 update tracks with associated objects
  UpdateAssignedTracks(&tracks_predict, &transformed_objects, assignments,
                       time_diff);
  // E.2 update tracks without associated objects
  UpdateUnassignedTracks(tracks_predict, unassigned_tracks, time_diff);
  DeleteLostTracks();
  // E.3 create new tracks for objects without associated tracks
  CreateNewTracks(transformed_objects, unassigned_objects);

  // F. collect tracked results
  CollectTrackedResults(tracked_objects);
  return true;
}

bool HmObjectTracker::InitializeTrack(
    const std::vector<std::shared_ptr<Object>>& objects,
    const double timestamp, const Eigen::Matrix4f& trans,
    std::vector<std::shared_ptr<Object>>* tracked_objects) 
  {
    // A. track setup
    Eigen::Matrix4f velo2world_pose = Eigen::Matrix4f::Identity();
    velo2world_pose=trans;
    global_to_local_offset_ = Eigen::Vector3f(
        -velo2world_pose(0, 3), -velo2world_pose(1, 3), -velo2world_pose(2, 3));

    // B. preprocessing
    // B.1 coordinate transformation
    TransformPoseGlobal2Local(&velo2world_pose);
    // cout << "velo2local_pose\n" << velo2world_pose<<endl;
    // B.2 construct tracked objects
    std::vector<std::shared_ptr<TrackedObject>> transformed_objects;
    ConstructTrackedObjects(objects, &transformed_objects, velo2world_pose,
                            trans);

    // C. create tracks
    std::vector<int> unassigned_objects;
    unassigned_objects.resize(transformed_objects.size());
    std::iota(unassigned_objects.begin(), unassigned_objects.end(), 0);
    CreateNewTracks(transformed_objects, unassigned_objects);
    time_stamp_ = timestamp;

    // D. collect tracked results
    CollectTrackedResults(tracked_objects);
    return true;
}

void HmObjectTracker::TransformPoseGlobal2Local(Eigen::Matrix4f* pose) 
{
  (*pose)(0, 3) += global_to_local_offset_(0);
  (*pose)(1, 3) += global_to_local_offset_(1);
  (*pose)(2, 3) += global_to_local_offset_(2);
}

void HmObjectTracker::ConstructTrackedObjects(
    const std::vector<std::shared_ptr<Object>>& objects,
    std::vector<std::shared_ptr<TrackedObject>>* tracked_objects,
    const Eigen::Matrix4f& pose, const Eigen::Matrix4f& trans) 
{
  int num_objects = objects.size();
  tracked_objects->clear();
  tracked_objects->resize(num_objects);
  for (int i = 0; i < num_objects; ++i) 
  {
    std::shared_ptr<Object> obj(new Object());
    obj->clone(*objects[i]);
    (*tracked_objects)[i].reset(new TrackedObject(obj));
    // Computing shape featrue
    if (use_histogram_for_match_) 
    {
      ComputeShapeFeatures(&((*tracked_objects)[i]));
    }
    // Transforming all tracked objects
    TransformTrackedObject(&((*tracked_objects)[i]), pose);
    // Setting barycenter as anchor point of tracked objects
    Eigen::Vector3f anchor_point = (*tracked_objects)[i]->barycenter;
    (*tracked_objects)[i]->anchor_point = anchor_point;
    // Getting lane direction of tracked objects
    // pcl_util::PointD query_pt;
    // query_pt.x = anchor_point(0) - global_to_local_offset_(0);
    // query_pt.y = anchor_point(1) - global_to_local_offset_(1);
    // query_pt.z = anchor_point(2) - global_to_local_offset_(2);
    // Eigen::Vector3f lane_dir;
    // if (!options.hdmap_input->GetNearestLaneDirection(query_pt, &lane_dir)) {
    //   AERROR << "Failed to initialize the lane direction of tracked object!";
    //   // Set lane dir as host dir if query lane direction failed
    //   lane_dir = (pose * Eigen::Vector4f(1, 0, 0, 0)).head(3);
    // }
    // (*tracked_objects)[i]->lane_direction = lane_dir.cast<float>();
  }
}

void HmObjectTracker::ComputeShapeFeatures(
    std::shared_ptr<TrackedObject>* obj) const
{
  // Compute object's shape feature
  std::shared_ptr<Object>& temp_object = (*obj)->object_ptr;
  FeatureDescriptor fd(temp_object->cloud);
  fd.ComputeHistogram(this->histogram_bin_size,
                      &temp_object->shape_features);
}

void HmObjectTracker::TransformTrackedObject(
    std::shared_ptr<TrackedObject>* obj, const Eigen::Matrix4f& pose) 
{
  // Transform tracked object with given pose
  TransformObject(&((*obj)->object_ptr), pose);
  // transform direction
  Eigen::Vector3f& dir = (*obj)->direction;
  dir =
      (pose * Eigen::Vector4f(dir(0), dir(1), dir(2), 0)).head(3).cast<float>();
  // transform center
  Eigen::Vector3f& center = (*obj)->center;
  center = (pose * Eigen::Vector4f(center(0), center(1), center(2), 1)).head(3)
               .cast<float>();
  // transform barycenter
  Eigen::Vector3f& barycenter = (*obj)->barycenter;
  barycenter =
      (pose * Eigen::Vector4f(barycenter(0), barycenter(1), barycenter(2), 1))
          .head(3)
          .cast<float>();
}

void HmObjectTracker::TransformObject(std::shared_ptr<Object>* obj,
                                      const Eigen::Matrix4f& pose) 
{
  // Transform object with given pose
  Eigen::Vector3f& dir = (*obj)->direction;
  dir = (pose * Eigen::Vector4f(dir[0], dir[1], dir[2], 0)).head(3);
  // transform center
  Eigen::Vector3f& center = (*obj)->center;
  center = (pose * Eigen::Vector4f(center[0], center[1], center[2], 1)).head(3);
  // transform cloud & polygon
  TransformPointCloud(pose, (*obj)->cloud);
  TransformPointCloud(pose, (*obj)->polygon);
}

void HmObjectTracker::ComputeTracksPredict(
    std::vector<Eigen::VectorXf>* tracks_predict, const double time_diff) 
{
  // Compute tracks' predicted states
  int no_track = object_tracks_.Size();
  tracks_predict->resize(no_track);
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.GetTracks();
  for (int i = 0; i < no_track; ++i) 
  {
    (*tracks_predict)[i] = tracks[i]->Predict(time_diff);
  }
}

void HmObjectTracker::UpdateAssignedTracks(
    std::vector<Eigen::VectorXf>* tracks_predict,
    std::vector<std::shared_ptr<TrackedObject>>* new_objects,
    const std::vector<std::pair<int, int>>& assignments,
    const double time_diff) 
{
  // Update assigned tracks
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.GetTracks();
  for (size_t i = 0; i < assignments.size(); ++i) 
  {
    int track_id = assignments[i].first;
    int obj_id = assignments[i].second;
    tracks[track_id]->UpdateWithObject(&(*new_objects)[obj_id], time_diff);
  }
}

void HmObjectTracker::UpdateUnassignedTracks
( const std::vector<Eigen::VectorXf>& tracks_predict,
    const std::vector<int>& unassigned_tracks, const double time_diff) 
{
  // Update tracks without matched objects
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.GetTracks();
  for (size_t i = 0; i < unassigned_tracks.size(); ++i) 
  {
    int track_id = unassigned_tracks[i];
    tracks[track_id]->UpdateWithoutObject(tracks_predict[track_id], time_diff);
  }
}

void HmObjectTracker::CreateNewTracks(
    const std::vector<std::shared_ptr<TrackedObject>>& new_objects,
    const std::vector<int>& unassigned_objects) 
{
  // Create new tracks for objects without matched tracks
  for (size_t i = 0; i < unassigned_objects.size(); ++i) 
  {
    int obj_id = unassigned_objects[i];
    ObjectTrackPtr track(new ObjectTrack(new_objects[obj_id]));
    object_tracks_.AddTrack(track);
  }
}

void HmObjectTracker::DeleteLostTracks() {
  // Delete lost tracks
  object_tracks_.RemoveLostTracks();
}

void HmObjectTracker::CollectTrackedResults(
    std::vector<std::shared_ptr<Object>>* tracked_objects) 
{
  // Collect tracked results for reporting include objects may be occluded
  // temporaryly
  const std::vector<ObjectTrackPtr>& tracks = object_tracks_.GetTracks();
  tracked_objects->resize(tracks.size());

  int track_number = 0;
  for (size_t i = 0; i < tracks.size(); ++i) 
  {
    if (tracks[i]->consecutive_invisible_count_ >
        this->collect_consecutive_invisible_maximum)
    {
      continue;
    } 
    if (tracks[i]->age_ < this->collect_age_minimum)
    {
      continue;
    }
    std::shared_ptr<Object> obj(new Object);
    std::shared_ptr<TrackedObject> result_obj = tracks[i]->current_object_;
    obj->clone(*(result_obj->object_ptr));
    // fill tracked information of object
    obj->direction = result_obj->direction.cast<float>();
    obj->length= result_obj->size[0];
    obj->width= result_obj->size[1];
    obj->height= result_obj->size[2];
    obj->velocity = result_obj->velocity.cast<float>();
    obj->velocity_uncertainty = result_obj->velocity_uncertainty.cast<float>();
    obj->track_id = tracks[i]->idx_;
    obj->tracking_time = tracks[i]->period_;
    obj->type = result_obj->type;
    obj->center = result_obj->center.cast<float>() - global_to_local_offset_;
    obj->anchor_point =result_obj->anchor_point.cast<float>() - global_to_local_offset_;
    // restore original world coordinates
    for (size_t j = 0; j < obj->cloud->size(); ++j) 
    {
      obj->cloud->points[j].x -= global_to_local_offset_[0];
      obj->cloud->points[j].y -= global_to_local_offset_[1];
      obj->cloud->points[j].z -= global_to_local_offset_[2];
    }
    for (size_t j = 0; j < obj->polygon.points.size();++j) 
    {
      obj->polygon.points[j].x -= global_to_local_offset_[0];
      obj->polygon.points[j].y -= global_to_local_offset_[1];
      obj->polygon.points[j].z -= global_to_local_offset_[2];
    }
    (*tracked_objects)[track_number] = obj;
    ++track_number;
  }
  tracked_objects->resize(track_number);
}
