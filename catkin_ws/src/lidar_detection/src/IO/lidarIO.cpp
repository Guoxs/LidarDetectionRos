//
// Created by guoxs on 2020/9/23.
//
#include "lidarIO.h"


//constructor:
template<typename PointT>
LidarIO<PointT>::LidarIO() = default;

//de-constructor:
template<typename PointT>
LidarIO<PointT>::~LidarIO() = default;

template<typename PointT>
void LidarIO<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+ file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr LidarIO<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> LidarIO<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(
            boost::filesystem::directory_iterator{dataPath},
            boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
