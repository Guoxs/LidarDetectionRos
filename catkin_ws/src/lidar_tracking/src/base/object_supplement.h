/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-10 16:12:11
 */

#ifndef OBJECT_SUPPLEMENT_H_
#define OBJECT_SUPPLEMENT_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "boost/circular_buffer.hpp"
#include "opencv2/opencv.hpp"

#include "types.h"


struct TrackStateVars {
  Eigen::Matrix4f process_noise = Eigen::Matrix4f::Identity();
  // Eigen::Matrix4f measure_noise = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f trans_matrix = Eigen::Matrix4f::Identity();
  bool initialized_ = false;
};

struct alignas(16) LidarFrameSupplement {
  static TrackStateVars state_vars;
};

typedef std::shared_ptr<LidarFrameSupplement> LidarFrameSupplementPtr;
typedef std::shared_ptr<const LidarFrameSupplement>
    LidarFrameSupplementConstPtr;

struct alignas(16) RadarSupplement {
  RadarSupplement();
  ~RadarSupplement();
  RadarSupplement(const RadarSupplement& rhs);
  RadarSupplement& operator=(const RadarSupplement& rhs);
  void clone(const RadarSupplement& rhs);

  // distance
  float range = 0.0f;
  // x -> forward, y -> left
  float angle = 0.0f;
  float relative_radial_velocity = 0.0f;
  float relative_tangential_velocity = 0.0f;
  float radial_velocity = 0.0f;
};
typedef std::shared_ptr<RadarSupplement> RadarSupplementPtr;
typedef std::shared_ptr<const RadarSupplement> RadarSupplementConstPtr;

struct alignas(16) RadarFrameSupplement {
  RadarFrameSupplement();
  ~RadarFrameSupplement();
  RadarFrameSupplement(const RadarFrameSupplement& rhs);
  RadarFrameSupplement& operator=(const RadarFrameSupplement& rhs);
  void clone(const RadarFrameSupplement& rhs);
  static TrackStateVars state_vars;
};

typedef std::shared_ptr<RadarFrameSupplement> RadarFrameSupplementPtr;
typedef std::shared_ptr<const RadarFrameSupplement>
    RadarFrameSupplementConstPtr;

struct alignas(16) CameraFrameSupplement {
  CameraFrameSupplement();
  ~CameraFrameSupplement();
  CameraFrameSupplement(const CameraFrameSupplement& rhs);
  CameraFrameSupplement& operator=(const CameraFrameSupplement& rhs);
  void clone(const CameraFrameSupplement& rhs);

  cv::Mat depth_map;
  cv::Mat label_map;
  cv::Mat lane_map;
  cv::Mat img_src;
  std::string source_topic;
  static TrackStateVars state_vars;
};

typedef std::shared_ptr<CameraFrameSupplement> CameraFrameSupplementPtr;

typedef std::shared_ptr<const CameraFrameSupplement>
    CameraFrameSupplementConstPtr;

struct alignas(16) CameraSupplement {
  CameraSupplement();
  ~CameraSupplement();
  CameraSupplement(const CameraSupplement& rhs);
  CameraSupplement& operator=(const CameraSupplement& rhs);
  void clone(const CameraSupplement& rhs);

  // upper-left corner: x1, y1
  Eigen::Vector2d upper_left;
  // lower-right corner: x2, y2
  Eigen::Vector2d lower_right;
  // local track id
  int local_track_id = -1;

  // 2Dto3D, pts8.resize(16), x, y...
  std::vector<float> pts8;

  // front box upper-left corner: x1, y1
  Eigen::Vector2d front_upper_left;
  // front box  lower-right corner: x2, y2
  Eigen::Vector2d front_lower_right;

  // front box upper-left corner: x1, y1
  Eigen::Vector2d back_upper_left;
  // front box  lower-right corner: x2, y2
  Eigen::Vector2d back_lower_right;

  std::vector<float> object_feature;

  // this is `alpha` angle from KITTI: Observation angle of object,
  // ranging [-pi..pi]
  double alpha = 0.0;
};

typedef std::shared_ptr<CameraSupplement> CameraSupplementPtr;
typedef std::shared_ptr<const CameraSupplement> CameraSupplementConstPtr;

typedef Eigen::Matrix4f MotionType;
struct alignas(16) VehicleStatus {
  float roll_rate = 0;
  float pitch_rate = 0;
  float yaw_rate = 0;
  float velocity = 0;
  float velocity_x = 0;
  float velocity_y = 0;
  float velocity_z = 0;
  double time_ts = 0;     // time stamp
  double time_d = 0;      // time stamp difference in image
  MotionType motion = MotionType::Identity();  // Motion Matrix
};

typedef boost::circular_buffer<VehicleStatus> MotionBuffer;
typedef std::shared_ptr<MotionBuffer> MotionBufferPtr;
typedef std::shared_ptr<const MotionBuffer> MotionBufferConstPtr;

struct alignas(16) Vehicle3DStatus {
  float yaw_delta;  // azimuth angle change
  float pitch_delta;
  float roll_delta;
  float velocity_x;          // east
  float velocity_y;          // north
  float velocity_z;          // up
  float time_t;              // time stamp
  float time_d;              // time stamp difference in image
  Eigen::Matrix4f motion3d;  // 3-d Motion Matrix
};

typedef boost::circular_buffer<Vehicle3DStatus> Motion3DBuffer;
typedef std::shared_ptr<Motion3DBuffer> Motion3DBufferPtr;
typedef std::shared_ptr<const Motion3DBuffer> Motion3DBufferConstPtr;


#endif  //OBJECT_SUPPLEMENT_H_
