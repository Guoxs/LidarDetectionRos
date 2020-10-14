/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-15 16:58:27
 */


#ifndef OBSTACLE_BASE_OBJECT_H_
#define OBSTACLE_BASE_OBJECT_H_

#include <memory>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/PolygonStamped.h>

#include "Eigen/Core"
#include "types.h"
#include "object_supplement.h"



class Object {
  public:
  Object();
  void clone(const Object& rhs);
  std::string ToString() const;
  float getAngle();
  // void AddFourCorners(PerceptionObstacle* pb_obj) const;
  // void Serialize(PerceptionObstacle* pb_obj) const;
  // void Deserialize(const PerceptionObstacle& pb_obs);

  // object id per frame
  int id = 0;
  // point cloud of the object
  PointCloudType::Ptr cloud;
  // convex hull of the object
  PointCloudType  polygon;

  // oriented boundingbox information
  // main direction
  Eigen::Vector3f direction = Eigen::Vector3f(1, 0, 0);
  // the yaw angle, theta = 0.0 <=> direction = (1, 0, 0)
  double theta = 0.0;
  // ground center of the object (cx, cy, z_min)
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  // size of the oriented bbox, length is the size in the main direction
  float length = 0.0;
  float width = 0.0;
  float height = 0.0;
  // shape feature used for tracking
  std::vector<float> shape_features;

  // foreground score/probability
  float score = 0.0;
  // foreground score/probability type
  ScoreType score_type = ScoreType::SCORE_CNN;

  // Object classification type.
  ObjectType type = ObjectType::TYPE_UNKNOWN;
  SensorType sensor_type = SensorType::SENSOR_INVALID;
  // Probability of each type, used for track type.
  std::vector<float> type_probs;

  // fg/bg flag
  bool is_background = false;

  // tracking information
  int track_id = 0;
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
  // age of the tracked object
  double tracking_time = 0.0;
  double latest_tracked_time = 0.0;
  double timestamp = 0.0;

  // stable anchor_point during time, e.g., barycenter
  Eigen::Vector3f anchor_point;

  // noise covariance matrix for uncertainty of position and velocity
  Eigen::Matrix3f position_uncertainty;
  Eigen::Matrix3f velocity_uncertainty;

  // modeling uncertainty from sensor level tracker
  Eigen::Matrix4f state_uncertainty = Eigen::Matrix4f::Identity();
  // Tailgating (trajectory of objects)
  std::vector<Eigen::Vector3f> drops;
  // CIPV
  bool b_cipv = false;
  // local lidar track id
  int local_lidar_track_id = -1;
  // local radar track id
  int local_radar_track_id = -1;
  // local camera track id
  int local_camera_track_id = -1;

  int local_v2x_track_id=-1;

  // local lidar track ts
  float local_lidar_track_ts = -1;
  // local radar track ts
  float local_radar_track_ts = -1;
  // local camera track ts
  float local_camera_track_ts = -1;

  // sensor particular suplplements, default nullptr
  RadarSupplementPtr radar_supplement = nullptr;
  CameraSupplementPtr camera_supplement = nullptr;
};

// Sensor single frame objects.
class SensorObjects {
  public:
  SensorObjects() 
  {
    sensor2world_pose = Eigen::Matrix4f::Zero();
    sensor2world_pose_static = Eigen::Matrix4f::Zero();
  }

  std::string ToString() const;

  // Transmit error_code to next subnode.
  // common::ErrorCode error_code = common::ErrorCode::OK;

  SensorType sensor_type = SensorType::SENSOR_INVALID;
  std::string sensor_id;
  double timestamp = 0.0;
  SeqId seq_num = 0;
  std::vector<std::shared_ptr<Object>> objects;
  Eigen::Matrix4f sensor2world_pose;
  Eigen::Matrix4f sensor2world_pose_static;
  // LaneObjectsPtr lane_objects;

  uint32_t cipv_index = -1;
  uint32_t cipv_track_id = -1;

  // sensor particular suplplements, default nullptr
  RadarFrameSupplementPtr radar_frame_supplement = nullptr;
  CameraFrameSupplementPtr camera_frame_supplement = nullptr;
};


#endif  // OBSTACLE_BASE_OBJECT_H_
