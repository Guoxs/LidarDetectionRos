/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-13 13:07:01
 */


#ifndef HUNGARIAN_MATCHER_H_
#define HUNGARIAN_MATCHER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "../tracked_obj/tracked_object.h"
#include "object_track.h"


class HungarianMatcher {
 public:
  HungarianMatcher() {}
  ~HungarianMatcher() {}

  // @brief set match distance maximum for matcher
  // @params[IN] match_distance_maximum: match distance maximum
  // @return true if set successfuly, otherwise return false
  static bool SetMatchDistanceMaximum(const float match_distance_maximum);

  // @brief match detected objects to tracks
  // @params[IN] objects: new detected objects for matching
  // @params[IN] tracks: maintaining tracks for matching
  // @params[IN] tracks_predict: predicted state of maintained tracks
  // @params[OUT] assignments: assignment pair of object & track
  // @params[OUT] unassigned_tracks: tracks without matched object
  // @params[OUT] unassigned_objects: objects without matched track
  // @return nothing
  void Match(std::vector<std::shared_ptr<TrackedObject>>* objects,
             const std::vector<ObjectTrackPtr>& tracks,
             const std::vector<Eigen::VectorXf>& tracks_predict,
             std::vector<std::pair<int, int>>* assignments,
             std::vector<int>* unassigned_tracks,
             std::vector<int>* unassigned_objects);

  // @brief match detected objects to tracks in component level
  // @params[IN] association_mat: association matrix of all objects to tracks
  // @params[IN] track_component: component of track
  // @params[IN] object_component: component of object
  // @params[OUT] sub_assignments: component assignment pair of object & track
  // @params[OUT] sub_unassigned_tracks: component tracks not matched
  // @params[OUT] sub_unasgined_objects: component objects not matched
  // @return nothing
  void MatchInComponents(const Eigen::MatrixXf& association_mat,
                         const std::vector<int>& track_component,
                         const std::vector<int>& obj_component,
                         std::vector<std::pair<int, int>>* sub_assignments,
                         std::vector<int>* sub_unassigned_tracks,
                         std::vector<int>* sub_unassigned_objects);

  std::string Name() const { return "HungarianMatcher"; }

 protected:
  // @brief compute association matrix
  // @params[IN] tracks: maintained tracks for matching
  // @params[IN] tracks_predict: predicted states of maintained tracks
  // @params[IN] new_objects: recently detected objects
  // @params[OUT] association_mat: matrix of association distance
  // @return nothing
  void ComputeAssociateMatrix(
      const std::vector<ObjectTrackPtr>& tracks,
      const std::vector<Eigen::VectorXf>& tracks_predict,
      const std::vector<std::shared_ptr<TrackedObject>>& new_objects,
      Eigen::MatrixXf* association_mat);

  // @brief compute connected components within given threshold
  // @params[IN] association_mat: matrix of association distance
  // @params[IN] connected_threshold: threshold of connected components
  // @params[OUT] track_components: connected objects of given tracks
  // @params[OUT] obj_components: connected tracks of given objects
  // @return nothing
  void ComputeConnectedComponents(
      const Eigen::MatrixXf& association_mat, const float connected_threshold,
      std::vector<std::vector<int>>* track_components,
      std::vector<std::vector<int>>* obj_components);

  // @brief assign objects to tracks using components
  // @params[IN] association_mat: matrix of association distance
  // @params[IN] assign_distance_maximum: threshold distance of assignment
  // @params[OUT] assignments: assignment pair of matched object & track
  // @params[OUT] unassigned_tracks: tracks without matched object
  // @params[OUT] unassigned_objects: objects without matched track
  // @return nothing
  void AssignObjectsToTracks(const Eigen::MatrixXf& association_mat,
                             const float assign_distance_maximum,
                             std::vector<std::pair<int, int>>* assignments,
                             std::vector<int>* unassigned_tracks,
                             std::vector<int>* unassigned_objects);

 private:
  // threshold of matching
  static float s_match_distance_maximum_;
};  // class HmMatcher


#endif  // HUNGARIAN_MATCHER_H_
