/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-10 12:39:11
 */


#ifndef HM_TRACKER_H_
#define HM_TRACKER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <ros/ros.h>

#include "object_track.h"
#include "../tracked_obj/tracked_object.h"
#include "hungarian_matcher.h"
#include "../base/object.h"


class HmObjectTracker {
 public:
      HmObjectTracker() = default;
      virtual ~HmObjectTracker() = default;

      // @brief initialize tracker's configs
      // @return true if initialize successfully, otherwise return false
      bool Init(const ros::NodeHandle& nh);

      // @brief track detected objects over consecutive frames
      // @params[IN] objects: recently detected objects
      // @params[IN] timestamp: timestamp of recently detected objects
      // @params[IN] options: tracker options with necessary information
      // @params[OUT] tracked_objects: tracked objects with tracking information
      // @return true if track successfully, otherwise return false
      bool Track(const std::vector<std::shared_ptr<Object>>& objects,
                 double timestamp, const Eigen::Matrix4f& trans,
                 std::vector<std::shared_ptr<Object>>* tracked_objects);

      // @brief get object tracks of tracker
      // @return object tracks maintained in tracker
      const std::vector<ObjectTrackPtr>& GetObjectTracks() const;

      std::string name() const
      {
          return "HmObjectTracker";
      }

     protected:
      // @brief initialize tracker after obtaining detection of first frame
      // @params[IN] objects: recently detected objects
      // @params[IN] timestamp: timestamp of recently detected objects
      // @params[IN] options: tracker options with necessary information
      // @params[OUT] tracked_objects: tracked objects with tracking information
      // @return true if initialize successfully, otherwise return false
      bool InitializeTrack(const std::vector<std::shared_ptr<Object>>& objects,
                           const double timestamp, const Eigen::Matrix4f& trans,
                           std::vector<std::shared_ptr<Object>>* tracked_objects);

      // @brief transform v2world pose to v2local pose intend to avoid huge value
      // float computing
      // @params[OUT] pose: v2world pose
      // @return nothing
      void TransformPoseGlobal2Local(Eigen::Matrix4f* pose);

      // @brief construct tracked objects via necessray transformation & feature
      // computing
      // @params[IN] objects: objects for construction
      // @params[OUT] tracked_objects: constructed objects
      // @params[IN] pose: pose using for coordinate transformation
      // @params[IN] options: tracker options with necessary information
      // @return nothing
      void ConstructTrackedObjects(
          const std::vector<std::shared_ptr<Object>>& objects,
          std::vector<std::shared_ptr<TrackedObject>>* tracked_objects,
          const Eigen::Matrix4f& pose, const Eigen::Matrix4f& trans);

      // @brief compute objects' shape feature
      // @params[OUT] object: object for computing shape feature
      // @return nothing
      void ComputeShapeFeatures(std::shared_ptr<TrackedObject>* obj) const;

      // @brief transform tracked object with given pose
      // @params[OUT] obj: tracked object for transfromation
      // @params[IN] pose: pose using for coordinate transformation
      // @return nothing
      void TransformTrackedObject(std::shared_ptr<TrackedObject>* obj,
                                  const Eigen::Matrix4f& pose);

      // @brief transform object with given pose
      // @params[OUT] obj: object for transfromation
      // @params[IN] pose: pose using for coordinate transformation
      // @return nothing
      void TransformObject(std::shared_ptr<Object>* obj,
                           const Eigen::Matrix4f& pose);

      // @brief compute predicted states of maintained tracks
      // @params[OUT] tracks_predict: predicted states of maintained tracks
      // @params[IN] time_diff: time interval for predicting
      // @return nothing
      void ComputeTracksPredict(std::vector<Eigen::VectorXf>* tracks_predict,
                                double time_diff);

      // @brief update assigned tracks
      // @params[IN] tracks_predict: predicted states of maintained tracks
      // @params[IN] new_objects: recently detected objects
      // @params[IN] assignments: assignment pair of <track, object>
      // @params[IN] time_diff: time interval for updating
      // @return nothing
      void UpdateAssignedTracks(
          std::vector<Eigen::VectorXf>* tracks_predict,
          std::vector<std::shared_ptr<TrackedObject>>* new_objects,
          const std::vector<std::pair<int, int>>& assignments,
          double time_diff);

      // @brief update tracks without matched objects
      // @params[IN] tracks_predict: predicted states of maintained tracks
      // @params[IN] unassigned_tracks: index of unassigned tracks
      // @params[IN] time_diff: time interval for updating
      // @return nothing
      void UpdateUnassignedTracks(
          const std::vector<Eigen::VectorXf>& tracks_predict,
          const std::vector<int>& unassigned_tracks, double time_diff);

      // @brief create new tracks for objects without matched track
      // @params[IN] new_objects: recently detected objects
      // @params[IN] unassigned_objects: index of unassigned objects
      // @return nothing
      void CreateNewTracks(
          const std::vector<std::shared_ptr<TrackedObject>>& new_objects,
          const std::vector<int>& unassigned_objects);

      // @brief delete lost tracks
      // @return nothing
      void DeleteLostTracks();

      // @brief collect tracked results
      // @params[OUT] tracked_objects: tracked objects with tracking information
      // @return nothing
      void CollectTrackedResults(
          std::vector<std::shared_ptr<Object>>* tracked_objects);

 private:
      // algorithm setup
      bool use_histogram_for_match_ = false;

      // matcher
      std::unique_ptr<HungarianMatcher> matcher_;

      // tracks
      ObjectTrackSet object_tracks_;

      // set offset to avoid huge value float computing
      Eigen::Vector3f global_to_local_offset_;
      double time_stamp_ = 0.0;
      bool valid_ = false;
      ros::NodeHandle nh_;
   
    //track_config
    int track_cached_history_size_maximum;
    int track_consecutive_invisible_maximum;
    float track_visible_ratio_minimum;
    int collect_age_minimum ;
    int collect_consecutive_invisible_maximum;
    float acceleration_noise_maximum;
    float speed_noise_maximum;
    float match_distance_maximum;
    float location_distance_weight;
    float direction_distance_weight;
    float bbox_size_distance_weight;
    float point_num_distance_weight;
    float histogram_distance_weight;
    int histogram_bin_size;
    bool use_adaptive;
    float measurement_noise;
    float initial_velocity_noise;
    float xy_propagation_noise;
    float z_propagation_noise;
    float breakdown_threshold_maximum;

};  // class HmObjectTracker

#endif  // HM_TRACKER_H_
