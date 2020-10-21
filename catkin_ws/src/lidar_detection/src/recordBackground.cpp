#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/filters/filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "IO/lidarIO.h"
#include "IO/lidarIO.cpp"
#include "processing/processPointClouds.h"
#include "processing/processPointClouds.cpp"

int main (int argc, char** argv)
{
    std::string bagRoot = argv[1];
    std::string bagName = argv[2];
    int backgroundNum = 50;
    std::string rootPath = "/home/guoxs/Documents/Project/Huituo/LidarDetectionRos/data/";

    rosbag::Bag bag;
    bag.open(rootPath + bagRoot + bagName + ".bag", rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.emplace_back("/rslidar_points");
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    pcl::PointCloud<pcl::PointXYZI>::Ptr backgroundCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>);

    //Stream PCD
    auto* cloudIO = new LidarIO<pcl::PointXYZI>();
    //processing PCD
    auto* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();

    rosbag::View::iterator it = view.begin();
    //indices for nan removing
    std::vector<int> indices;

    while ((backgroundNum > 0) & (it != view.end())){
        sensor_msgs::PointCloud2::ConstPtr input = (*it).instantiate<sensor_msgs::PointCloud2>();
        if (input != nullptr)
        {
            pcl::fromROSMsg(*input,*tempCloud);
            tempCloud->is_dense = false;
            pcl::removeNaNFromPointCloud(*tempCloud, *tempCloud, indices);
            *backgroundCloud = *backgroundCloud + *tempCloud;
        }
        backgroundNum--;
        it ++;
    }
    //voxel filter
    backgroundCloud = pointProcessor->voxelFilter(backgroundCloud, 0.3);
    //save background point cloud
    std::cout << "Saving background file..." << std::endl;
    std::string outputPath = rootPath + bagRoot + "background.pcd";
    cloudIO->savePcd(backgroundCloud, outputPath);
}

