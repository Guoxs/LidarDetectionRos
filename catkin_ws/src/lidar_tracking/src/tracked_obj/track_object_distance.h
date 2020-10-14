/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-10 16:41:22
 */


#ifndef TRACK_OBJECT_DISTANCE_H_
#define TRACK_OBJECT_DISTANCE_H_

#include <memory>
#include <string>

#include "Eigen/Core"
#include "../filter/object_track.h"
#include "tracked_object.h"


class TrackObjectDistance {
 public:
  // @brief set weight of location dist for all the track object distance
  // objects
  // @params[IN] location_distance_weight: weight of location dist
  // @return true if set successfully, otherwise return false
  static bool SetLocationDistanceWeight(const float location_distance_weight);

  // @brief set weight of direction dist for all the track object distance
  // objects
  // @params[IN] direction_distance_weight: weight of direction dist
  // @return true if set successfully, otherwise return false
  static bool SetDirectionDistanceWeight(
      const float direction_distance_weight);

  // @brief set weight of bbox size dist for all the track object distance
  // objects
  // @params[IN] bbox_size_distance_weight: weight of bbox size dist
  // @return true if set successfully, otherwise return false
  static bool SetBboxSizeDistanceWeight(const float bbox_size_distance_weight);

  // @brief set weight of point num dist for all the track object distance
  // objects
  // @params[IN] point_num_distance_weight: weight of point num dist
  // @return true if set successfully, otherwise return false
  static bool SetPointNumDistanceWeight(const float point_num_distance_weight);

  // @brief set weight of histogram dist for all the track object distance
  // objects
  // @params[IN] weight_histogram_dist: weight of histogram dist
  // @return true if set successfully, otherwise return false
  static bool SetHistogramDistanceWeight(
      const float histogram_distance_weight);

  // @brief compute distance for given track & object
  // @params[IN] track: track for <track, object> distance computing
  // @params[IN] track_predict: predicted state of given track
  // @params[IN] new_object: recently detected object
  // @return computed <track, object> distance
  static float ComputeDistance(
      ObjectTrackPtr track, const Eigen::VectorXf& track_predict,
      const std::shared_ptr<TrackedObject>& new_object);

  std::string Name() const { return "TrackObjectDistance"; }

 private:
  // @brief compute location distance for given track & object
  // @params[IN] track: track for <track, object> distance computing
  // @params[IN] track_predict: predicted state of given track
  // @params[IN] new_object: recently detected object
  // @return location distacne of given <track, object>
  static float ComputeLocationDistance(
      ObjectTrackPtr track, const Eigen::VectorXf& track_predict,
      const std::shared_ptr<TrackedObject>& new_object);

  // @brief compute direction distance for given track & object
  // @params[IN] track: track for <track, object> distance computing
  // @params[IN] track_predict: predicted state of given track
  // @params[IN] new_object: recently detected object
  // @return direction distance of given <track, object>
  static float ComputeDirectionDistance(
      ObjectTrackPtr track, const Eigen::VectorXf& track_predict,
      const std::shared_ptr<TrackedObject>& new_object);

  // @brief compute bbox size distance for given track & object
  // @params[IN] track: track for <track, object> distance computing
  // @params[IN] new_object: recently detected object
  // @return bbox size distance of given <track, object>
  static float ComputeBboxSizeDistance(
      ObjectTrackPtr track,
      const std::shared_ptr<TrackedObject>& new_object);

  // @brief compute point num distance for given track & object
  // @params[IN] track: track for <track, object> distance computing
  // @params[IN] new_object: recently detected object
  // @return point num distance of given <track, object>
  static float ComputePointNumDistance(
      ObjectTrackPtr track,
      const std::shared_ptr<TrackedObject>& new_object);

  // @brief compute histogram distance for given track & object
  // @params[IN] track: track for <track, object> distance computing
  // @params[IN] new_object: recently detected object
  // @return histogram distance of given <track, object>
  static float ComputeHistogramDistance(
      ObjectTrackPtr track,
      const std::shared_ptr<TrackedObject>& new_object);

 protected:
  // distance weights
  static double s_location_distance_weight_;
  static double s_direction_distance_weight_;
  static double s_bbox_size_distance_weight_;
  static double s_point_num_distance_weight_;
  static double s_histogram_distance_weight_;

 
};  // class TrackObjectDistance


#endif  // TRACK_OBJECT_DISTANCE_H_
