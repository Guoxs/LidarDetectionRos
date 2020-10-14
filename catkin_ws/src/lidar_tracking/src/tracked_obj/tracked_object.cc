/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-10 15:18:45
 */


#include "tracked_object.h"

#include "../common/geometry_util.h"


TrackedObject::TrackedObject(std::shared_ptr<Object> obj_ptr)
    : object_ptr(obj_ptr) 
{
  if (object_ptr != nullptr) 
  {
    barycenter = GetCloudBarycenter<pcl::PointXYZI>(object_ptr->cloud)
                     .cast<float>();
    center = object_ptr->center.cast<float>();
    size = Eigen::Vector3f(object_ptr->length, object_ptr->width,
                           object_ptr->height);
    direction = object_ptr->direction.cast<float>();
    lane_direction = Eigen::Vector3f::Zero();
    anchor_point = barycenter;
    velocity = Eigen::Vector3f::Zero();
    acceleration = Eigen::Vector3f::Zero();
    type = object_ptr->type;
    velocity_uncertainty = Eigen::Matrix3f::Identity() * 5;
  }
}

void TrackedObject::clone(const TrackedObject& rhs) 
{
  *this = rhs;
  object_ptr.reset(new Object());
  object_ptr->clone(*rhs.object_ptr);
}
