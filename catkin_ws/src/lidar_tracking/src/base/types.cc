/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-15 14:12:24
 */


#include "types.h"

std::string GetObjectName(const ObjectType& obj_type) {
  std::string obj_name;
  switch (obj_type) {
    case ObjectType::TYPE_UNKNOWN:
      obj_name = "unknown";
      break;
    case ObjectType::TYPE_PEDESTRIAN:
      obj_name = "pedestrian";
      break;
    case ObjectType::TYPE_CAR:
      obj_name = "car";
      break;
    case ObjectType::TYPE_DUMPER:
      obj_name = "dumper";
      break;
    case ObjectType::TYPE_EXCAVATOR:
      obj_name = "excavator";
      break;
    case ObjectType::TYPE_FLUSHER:
      obj_name = "flusher";
      break;
    case ObjectType::TYPE_GRADER:
      obj_name="grader";
      break;
    case ObjectType::TYPE_DOZER:
      obj_name="dozer";
      break;
    default:
      obj_name = "error";
      break;
  }
  return obj_name;
}

std::string GetSensorType(SensorType sensor_type) {
  switch (sensor_type) 
  {
    case SensorType::SENSOR_LIDAR:
      return "lidar";
    case SensorType::SENSOR_CAMERA_LEFT:
      return "camera_left";
    case SensorType::SENSOR_CAMERA_MID:
      return "camera_mid";
    case SensorType::SENSOR_CAMERA_RIGHT:
      return "camera_right";
    case SensorType::SENSOR_RADAR:
      return "radar";
    // case SensorType::ULTRASONIC:
      // return "ultrasonic";
    case SensorType::SENSOR_V2X:
      return "v2x";
    case SensorType::SENSOR_INVALID:
      return "unknown_sensor_type";
  }
  return "";
}

bool is_lidar(SensorType sensor_type) 
{
  return (sensor_type == SensorType::SENSOR_LIDAR);
}

bool is_radar(SensorType sensor_type) 
{
  return (sensor_type == SensorType::SENSOR_RADAR);
}

bool is_camera(SensorType sensor_type) 
{
  return (sensor_type == SensorType::SENSOR_CAMERA_LEFT || sensor_type==SensorType::SENSOR_CAMERA_MID
  || sensor_type==SensorType::SENSOR_CAMERA_RIGHT);
}

bool is_v2x(SensorType sensor_type) 
{
  return (sensor_type == SensorType::SENSOR_V2X);
}

