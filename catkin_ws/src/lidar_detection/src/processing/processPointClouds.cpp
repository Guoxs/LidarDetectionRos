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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::RANSAC(
        const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
        int maxInter, float distanceThreshold)
{
    typename pcl::PointCloud<PointT>::Ptr foreground(new pcl::PointCloud<PointT>);

    auto startTime = std::chrono::steady_clock::now();

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setMaxIterations(maxInter);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(inputCloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty())
    {
        std::cout<<"error! Could not found any inliers!"<< std::endl;
    }
    // extract ground
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(inputCloud);
    extractor.setIndices(inliers);
    extractor.setNegative(true);
    extractor.filter(*foreground);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

    return foreground;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::PMorphologicalFilter(
        const typename pcl::PointCloud<PointT>::Ptr& inputCloud,
        float max_window_size, float slope, float max_distance,
        float initial_distance, float cell_size, float base, bool exponential)
{
    // Compute the series of window sizes and height thresholds
    std::vector<float> height_thresholds;
    std::vector<float> window_sizes;
    std::vector<int> ground_indices;
    int iteration = 0;
    float window_size = 0.0f;
    float height_threshold = 0.0f;

    auto startTime = std::chrono::steady_clock::now();

    while (window_size < max_window_size)
    {
        // Determine the initial window size.
        if (exponential)
            window_size = cell_size * (2.0f * std::pow (base, iteration) + 1.0f);
        else
            window_size = cell_size * (2.0f * (iteration+1) * base + 1.0f);
        std::cout << "window_size  " << window_size  << std::endl;
        // Calculate the height threshold to be used in the next iteration.
        if (iteration == 0)
            height_threshold = initial_distance;
        else
            height_threshold = slope * (window_size - window_sizes[iteration-1]) * cell_size + initial_distance;
        std::cout << "height_threshold  " << height_threshold  << std::endl;

        // Enforce max distance on height threshold
        if (height_threshold > max_distance)
            height_threshold = max_distance;

        window_sizes.push_back (window_size);
        height_thresholds.push_back (height_threshold);

        iteration++;
    }
    // Ground indices are initially limited to those points in the input cloud we
    // wish to process
    for (int i=0;i< inputCloud->points.size();i++){
        ground_indices.push_back(i);
    }

    // Progressively filter ground returns using morphological open
    for (size_t i = 0; i < window_sizes.size (); ++i)
    {
        std::cout<< "Iteration " << i << "height threshold = " << height_thresholds[i] << " window size = " <<
            window_sizes[i] << std::endl;

        // Limit filtering to those points currently considered ground returns
        typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        pcl::copyPointCloud<PointT> (*inputCloud, ground_indices, *cloud);

        // Create new cloud to hold the filtered results. Apply the morphological
        // opening operation at the current window size.
        typename pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
        pcl::applyMorphologicalOperator<PointT> (cloud, window_sizes[i], pcl::MORPH_OPEN, *cloud_f);

        // Find indices of the points whose difference between the source and
        // filtered point clouds is less than the current height threshold.
        std::vector<int> pt_indices;
        //cout << "ground.size() = " << ground.size() << endl;
        for (size_t p_idx = 0; p_idx < ground_indices.size (); ++p_idx)
        {
            float diff = cloud->points[p_idx].z - cloud_f->points[p_idx].z;
            //cout << "diff " << diff << endl;
            if (diff < height_thresholds[i])
                pt_indices.push_back (ground_indices[p_idx]);
        }

        // Ground is now limited to pt_indices
        ground_indices.swap (pt_indices);
        std::cout << "ground now has " << ground_indices.size () << " points" << std::endl;
    }
    typename pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);
    // Extract cloud_in with ground indices
    pcl::copyPointCloud<PointT> (*inputCloud, ground_indices, *cloud_out);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "progressive morphological filter took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_out;
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

    auto startTime = std::chrono::steady_clock::now();

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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "OBB took " << elapsedTime.count() << " milliseconds for each cluster" << std::endl;

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
    auto startTime = std::chrono::steady_clock::now();

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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "MinBox took " << elapsedTime.count() << " milliseconds for each cluster" << std::endl;

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
