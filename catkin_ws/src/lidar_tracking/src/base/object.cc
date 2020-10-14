/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-12 12:36:37
 */


#include "object.h"
#include <pcl/common/io.h>

using Eigen::Vector3f;

Object::Object() 
{
  cloud.reset(new PointCloudType);
  type_probs.resize(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0);
  position_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
  velocity_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
}

void Object::clone(const Object& rhs) 
{
  *this = rhs;
  pcl::copyPointCloud<PointType, PointType>(*(rhs.cloud), *cloud);
  // pcl::copyPointCloud<PointType,PointType>(rhs.polygon,polygon);
  radar_supplement = nullptr;
  if (rhs.radar_supplement != nullptr) {
    radar_supplement.reset(new RadarSupplement(*rhs.radar_supplement));
  }
  camera_supplement = nullptr;
  if (rhs.camera_supplement != nullptr) {
    camera_supplement.reset(new CameraSupplement());
    camera_supplement->clone(*(rhs.camera_supplement));
  }
}

  float Object::getAngle()
  {
    float angle;
    if (fabs(direction[0]) < DBL_MIN) 
    {
      angle = direction(1) > 0 ? M_PI / 2 : -M_PI / 2;
    } else 
    {
      angle=atan2(direction[1],direction[0]);
    }
    return angle;
  }




