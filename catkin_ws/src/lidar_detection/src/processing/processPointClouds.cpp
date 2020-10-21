//
// Created by guoxs on 2020/9/23.
//
#include "processPointClouds.h"
#include "../minbox/minBox.cpp"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() = default;

//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() = default;

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::BoxFilter(
        const typename pcl::PointCloud<PointT>::Ptr cloud,
        const Eigen::Vector4f minPoint,
        const Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // interesting region
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud);
    region.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "box filter took " << elapsedTime.count() << " milliseconds" << std::endl;
    return cloudRegion;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::voxelFilter(
        const typename pcl::PointCloud<PointT>::Ptr cloud,
        const float filterRes)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*cloudFiltered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "voxel filter took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudFiltered;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::radiusFilter(
        const typename pcl::PointCloud<PointT>::Ptr cloud,
        const float radius,
        const int min_pts)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_filter(new pcl::PointCloud<PointT>);
    pcl::RadiusOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setRadiusSearch(radius);
    sor.setMinNeighborsInRadius(min_pts);
    sor.setNegative(true);
    sor.filter(*cloud_filter);
    return cloud_filter;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::bkgRemove(
        const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
        const typename pcl::PointCloud<PointT>::Ptr& filteredCloudBGK,
        double radius, int minNum)
{
    typename pcl::PointCloud<PointT>::Ptr foreground(new pcl::PointCloud<PointT>);

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (filteredCloudBGK);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    auto startTime = std::chrono::steady_clock::now();

    for (const auto & searchPoint : inputCloud->points) {
        if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) < minNum){
            foreground->points.push_back(searchPoint);
        }
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "removing background took " << elapsedTime.count() << " milliseconds" << std::endl;

    return foreground;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::dustRemove(
        const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
        float distanceThreshold, float intensityThreshold)
{
    typename pcl::PointCloud<PointT>::Ptr foreground(new pcl::PointCloud<PointT>);

    auto startTime = std::chrono::steady_clock::now();

    for (const auto & searchPoint : inputCloud->points) {
        float distance = sqrt(pow(searchPoint.x, 2) + pow(searchPoint.y, 2) + pow(searchPoint.z, 2));
        if ((searchPoint.intensity <= intensityThreshold) || (distance >= distanceThreshold)){
                foreground->points.push_back(searchPoint);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "removing dust took " << elapsedTime.count() << " milliseconds" << std::endl;

    return foreground;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::DBSCANCluster(
        const typename pcl::PointCloud<PointT>::Ptr cloud,
        int core_point_min_pts, float tolerance, int min_size, int max_size)
{
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    std::vector<pcl::PointIndices> cluster_indices;
    DBSCANKdtreeCluster<PointT> ec;

    tree->setInputCloud(cloud);
    ec.setCorePointMinPts(core_point_min_pts);
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for(const pcl::PointIndices& getIndices: cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for(int index : getIndices.indices)
            cloudCluster-> points.push_back(cloud->points[index]);
        cloudCluster -> width = cloudCluster -> points.size();
        cloudCluster -> height = 1;
        cloudCluster -> is_dense = true;
        if (cloudCluster->width > 4) {
            clusters.push_back(cloudCluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "DBSCAN cluster took " << elapsedTime.count() << " milliseconds and found " <<
              clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(
        typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for(const pcl::PointIndices& getIndices: cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for(int index : getIndices.indices)
            cloudCluster-> points.push_back(cloud->points[index]);
        cloudCluster -> width = cloudCluster -> points.size();
        cloudCluster -> height = 1;
        cloudCluster -> is_dense = true;
        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Euclidean cluster took " << elapsedTime.count() << " milliseconds and found " <<
              clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::boundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::boundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    // OBB
    PointT min_point_OBB;
    PointT max_point_OBB;
    PointT position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    //copy point cloud
    typename pcl::PointCloud<PointT>::Ptr cpCluster(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cluster, *cpCluster);
    //set height to 0
    for(int nIndex = 0; nIndex < cpCluster->points.size(); nIndex++) {
        cpCluster->points[nIndex].z = 0;
    }

    pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
    feature_extractor.setInputCloud(cpCluster);
    feature_extractor.compute();

    feature_extractor.getOBB(min_point_OBB,max_point_OBB, position_OBB, rotational_matrix_OBB);

    Eigen::Vector3f trans = {position_OBB.x, position_OBB.y, (maxPoint.z + minPoint.z) / 2};
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    float cube_length = max_point_OBB.x - min_point_OBB.x;
    float cube_width = max_point_OBB.y - min_point_OBB.y;
//    float cube_height = max_point_OBB.z - min_point_OBB.z;

    BoxQ box;
    box.bboxTransform = trans;
    box.bboxQuaternion = quat;
    box.cube_height = maxPoint.z - minPoint.z;
    box.cube_length = cube_length;
    box.cube_width = cube_width;
    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::minBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    //copy point cloud
    typename pcl::PointCloud<PointT>::Ptr Cluster2D(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cluster, *Cluster2D);
    //set height to min z
    for(int nIndex = 0; nIndex < Cluster2D->points.size(); nIndex++) {
        Cluster2D->points[nIndex].z = minPoint.z;
    }

    //calculate 2D hull
    typename pcl::PointCloud<PointT>::Ptr planeHull(new pcl::PointCloud<PointT>);
    calculate2DHull(Cluster2D, planeHull);

    // calculate minBox
    BoxQ box;
    Minbox<PointT> minBox;
    minBox.ReconstructPolygon(planeHull, box);
    // calculate bboxQuaternion
    Eigen::Matrix3f rotational_matrix;
    rotational_matrix << box.direction[0], -box.direction[1], 0.0,
                         box.direction[1], box.direction[0], 0.0,
                         0.0, 0.0, 1.0;
    Eigen::Quaternionf quat(rotational_matrix);
    box.bboxQuaternion = quat;

    box.cube_height = maxPoint.z - minPoint.z;
    box.bboxTransform[2] += 0.5 * box.cube_height;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::calculate2DHull(
        typename pcl::PointCloud<PointT>::Ptr Cluster2D,
        typename pcl::PointCloud<PointT>::Ptr planeHull)
{
    ConvexHull2DXY<PointT> hull;
    hull.setInputCloud(Cluster2D);
    hull.setDimension(2);
    std::vector<pcl::Vertices> poly_vt;
    hull.Reconstruct2dxy (planeHull, &poly_vt);
}
