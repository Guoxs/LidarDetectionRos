/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-10 18:14:32
 */


#ifndef MODULES_PERCEPTION_OBSTACLE_COMMON_GEOMETRY_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_COMMON_GEOMETRY_UTIL_H_

#include <cfloat>

#include <algorithm>
#include <vector>
#include "Eigen/Core"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

#include "../base/types.h"


void ConvertPolygon2Cloud(const geometry_msgs::Polygon& polygon, PointCloudType& cloud);
void ConvertCloud2Polygon(const PointCloudType& cloud,geometry_msgs::Polygon& polygon);

void TransformPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                         const std::vector<int>& indices,
                         pcl::PointCloud<pcl::PointXYZI>* trans_cloud);


void TransformPointCloud(const Eigen::Matrix4f& trans_mat,
                         geometry_msgs::PolygonStamped* cloud_in_out);

void TransformPointCloud(const Eigen::Matrix4f& trans_mat,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_out);

void TransformPointCloud(const Eigen::Matrix4f& trans_mat,
pcl::PointCloud<pcl::PointXYZI>& cloud_in_out);

 /*
 * Other point cloud related methods
 * */
template <typename PointT>
void GetCloudMinMax3D(typename pcl::PointCloud<PointT>::Ptr cloud,
                      Eigen::Vector4f* min_point, Eigen::Vector4f* max_point) 
{
  Eigen::Vector4f& min_pt = *min_point;
  Eigen::Vector4f& max_pt = *max_point;
  min_pt[0] = min_pt[1] = min_pt[2] = FLT_MAX;
  max_pt[0] = max_pt[1] = max_pt[2] = -FLT_MAX;
  if (cloud->is_dense) {
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      min_pt[0] = std::min(min_pt[0], cloud->points[i].x);
      max_pt[0] = std::max(max_pt[0], cloud->points[i].x);
      min_pt[1] = std::min(min_pt[1], cloud->points[i].y);
      max_pt[1] = std::max(max_pt[1], cloud->points[i].y);
      min_pt[2] = std::min(min_pt[2], cloud->points[i].z);
      max_pt[2] = std::max(max_pt[2], cloud->points[i].z);
    }
  } else {
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      if (!pcl_isfinite(cloud->points[i].x) ||
          !pcl_isfinite(cloud->points[i].y) ||
          !pcl_isfinite(cloud->points[i].z)) {
        continue;
      }
      min_pt[0] = std::min(min_pt[0], cloud->points[i].x);
      max_pt[0] = std::max(max_pt[0], cloud->points[i].x);
      min_pt[1] = std::min(min_pt[1], cloud->points[i].y);
      max_pt[1] = std::max(max_pt[1], cloud->points[i].y);
      min_pt[2] = std::min(min_pt[2], cloud->points[i].z);
      max_pt[2] = std::max(max_pt[2], cloud->points[i].z);
    }
  }
}

template <typename PointT>
void ComputeBboxSizeCenter(typename pcl::PointCloud<PointT>::Ptr cloud,
                           const Eigen::Vector3f& direction,
                           Eigen::Vector3f* size, Eigen::Vector3f* center) 
{
  Eigen::Vector3f dir(direction[0], direction[1], 0);
  dir.normalize();
  Eigen::Vector3f ortho_dir(-dir[1], dir[0], 0.0);
  Eigen::Vector3f z_dir(dir.cross(ortho_dir));
  Eigen::Vector3f min_pt(DBL_MAX, DBL_MAX, DBL_MAX);
  Eigen::Vector3f max_pt(-DBL_MAX, -DBL_MAX, -DBL_MAX);

  Eigen::Vector3f loc_pt;
  for (std::size_t i = 0; i < cloud->size(); i++) 
  {
    Eigen::Vector3f pt = Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y,
                                         cloud->points[i].z);
    loc_pt[0] = pt.dot(dir);
    loc_pt[1] = pt.dot(ortho_dir);
    loc_pt[2] = pt.dot(z_dir);
    for (int j = 0; j < 3; j++) 
    {
      min_pt[j] = std::min(min_pt[j], loc_pt[j]);
      max_pt[j] = std::max(max_pt[j], loc_pt[j]);
    }
  }
  *size = max_pt - min_pt;
  *center = dir * ((max_pt[0] + min_pt[0]) * 0.5) +
            ortho_dir * ((max_pt[1] + min_pt[1]) * 0.5) + z_dir * (min_pt[2]+max_pt[2])*0.5;
}

template <typename PointT>
Eigen::Vector3f GetCloudBarycenter(
    typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  int point_num = cloud->points.size();
  Eigen::Vector3f barycenter(0, 0, 0);

  for (int i = 0; i < point_num; i++) {
    const PointT& pt = cloud->points[i];
    barycenter[0] += pt.x;
    barycenter[1] += pt.y;
    barycenter[2] += pt.z;
  }

  if (point_num > 0) {
    barycenter[0] /= point_num;
    barycenter[1] /= point_num;
    barycenter[2] /= point_num;
  }
  return barycenter;
}

void TransAffineToMatrix4(const Eigen::Vector3f& translation,
                          const Eigen::Vector4f& rotation,
                          Eigen::Matrix4f* trans_matrix);

void ComputeMostConsistentBboxDirection(const Eigen::Vector3f& previous_dir,
                                        Eigen::Vector3f* current_dir);

double VectorCosTheta2dXy(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

double VectorTheta2dXy(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);


#endif  // GEOMETRY_UTIL_H_
