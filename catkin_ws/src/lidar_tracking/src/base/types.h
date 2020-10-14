/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-07-03 17:33:32
 */


#ifndef OBSTACLE_BASE_TYPES_H_
#define OBSTACLE_BASE_TYPES_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudType;


struct grid{
        int num;
        float z;
        float z_ave;
        float z_min;
        float z_max;
        };

enum ObjectType {
  TYPE_UNKNOWN = 0,
  TYPE_PEDESTRIAN = 1,
  TYPE_CAR = 2,
  TYPE_TRUCK = 3,
  TYPE_DUMPER = 4,
  TYPE_EXCAVATOR = 5,
  TYPE_FLUSHER = 6,
  TYPE_GRADER = 7,
  TYPE_DOZER = 8,
  MAX_OBJECT_TYPE=9,
};
enum InternalObjectType {
  INT_BACKGROUND = 0,
  INT_SMALLMOT = 1,
  INT_PEDESTRIAN = 2,
  INT_NONMOT = 3,
  INT_BIGMOT = 4,
  INT_UNKNOWN = 5,
  INT_MAX_OBJECT_TYPE = 6,
};

enum SensorType 
{
  SENSOR_INVALID = 0,
  SENSOR_LIDAR = 1,
  SENSOR_CAMERA_LEFT = 2,
  SENSOR_CAMERA_MID = 3,
  SENSOR_CAMERA_RIGHT = 4,
  SENSOR_RADAR = 5,
  SENSOR_V2X=6,
};

enum class ScoreType 
{
  UNKNOWN_SCORE_TYPE = 0,
  SCORE_CNN = 1,
  SCORE_RADAR = 2,
};


using SeqId = uint32_t;

std::string GetObjectName(const ObjectType& obj_type);

std::string GetSensorType(SensorType sensor_type);

bool is_lidar(SensorType sensor_type);
bool is_radar(SensorType sensor_type);
bool is_camera(SensorType sensor_type);
bool is_v2x(SensorType sensor_type);

#endif  // OBSTACLE_BASE_TYPES_H_
