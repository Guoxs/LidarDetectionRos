//
// Created by guoxs on 2020/9/23.
//
#ifndef LIDARDETECTION_PROCESSPOINTCLOUDS_H
#define LIDARDETECTION_PROCESSPOINTCLOUDS_H

#include <chrono>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include "../DBSCAN/DBSCAN_kdtree.h"
#include "../render/box.h"
#include "../minbox/convex_hullxy.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();

    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr BoxFilter(
            typename pcl::PointCloud<PointT>::Ptr cloud,
            Eigen::Vector4f minPoint,
            Eigen::Vector4f maxPoint);

    typename pcl::PointCloud<PointT>::Ptr voxelFilter(
            typename pcl::PointCloud<PointT>::Ptr cloud,
            float filterRes);

    typename pcl::PointCloud<PointT>::Ptr radiusFilter(
            typename pcl::PointCloud<PointT>::Ptr cloud,
            float radius, int min_pts);

    typename pcl::PointCloud<PointT>::Ptr bkgRemove(
            const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
            const typename pcl::PointCloud<PointT>::Ptr& filteredCloudBGK,
            double radius, int minNum);

    typename pcl::PointCloud<PointT>::Ptr dustRemove(
            const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
            float distanceThreshold, float intensityThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> DBSCANCluster(
            typename pcl::PointCloud<PointT>::Ptr cloud,
            int core_point_min_pts, float tolerance, int min_size, int max_size);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(
            typename pcl::PointCloud<PointT>::Ptr cloud,
            float clusterTolerance, int minSize, int maxSize);

    Box boundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    BoxQ boundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster);

    BoxQ minBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster);

    void calculate2DHull(
            typename pcl::PointCloud<PointT>::Ptr Cluster2D,
            typename pcl::PointCloud<PointT>::Ptr planeHull);
};

#endif //LIDARDETECTION_PROCESSPOINTCLOUDS_H
