//
// Created by guoxs on 2020/9/23.
//

#ifndef LIDARDETECTION_LIDARIO_H
#define LIDARDETECTION_LIDARIO_H

#include <pcl/io/pcd_io.h>
#include <string>

template<typename PointT>
class LidarIO{
public:
    //constructor
    LidarIO();
    //deconstructor
    ~LidarIO();

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};

#endif //LIDARDETECTION_LIDARIO_H
