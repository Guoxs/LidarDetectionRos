/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-15 13:49:05
 */

#ifndef HM_TRACKER_TRACKED_OBJECT_H_
#define HM_TRACKER_TRACKED_OBJECT_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "../base/object.h"

struct TrackedObject {
  /* NEED TO NOTICE: All the states of track would be collected mainly based on
   * the states of tracked object. Thus, update tracked object's state when you
   * update the state of track !!! */
  TrackedObject() = default;
  explicit TrackedObject(std::shared_ptr<Object> obj_ptr);

  // deep copy (copy point clonds)
  void clone(const TrackedObject& rhs);

  // cloud
  // store transformed object before tracking
  std::shared_ptr<Object> object_ptr;

  Eigen::Vector3f barycenter;

  // bbox
  Eigen::Vector3f center;
  Eigen::Vector3f size;
  Eigen::Vector3f direction;
  Eigen::Vector3f lane_direction;

  // states
  Eigen::Vector3f anchor_point;
  Eigen::Vector3f velocity;
  Eigen::Matrix3f velocity_uncertainty;
  Eigen::Vector3f acceleration;

  // class type
  ObjectType type;

  // association distance
  // range from 0 to association_score_maximum
  float association_score = 0.0f;
};  // struct TrackedObject


#endif
