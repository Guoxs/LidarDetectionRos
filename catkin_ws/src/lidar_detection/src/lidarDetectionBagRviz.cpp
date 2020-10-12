//
// Created by guoxs on 2020/10/12.
//
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include "global.h"
#include "msg_util.cpp"
#include "IO/lidarIO.h"
#include "IO/lidarIO.cpp"
#include "processing/processPointClouds.h"
#include "processing/processPointClouds.cpp"


void lidarDetection(ProcessPointClouds<pcl::PointXYZI>* PointProcessor,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredBgCloud,
                    waytous_perception_msgs::ObjectArray& lidar_detection_info);


int main (int argc, char** argv)
{
    ros::init (argc, argv, "lidar_detection_bag");
    ros::NodeHandle nh;

    std::string bagName = argv[1];
    std::string rootPath = "/home/guoxs/Documents/Project/Huituo/LidarDetectionRos/data/";

    std::cout << "Starting Detecting..." << std::endl;

    rosbag::Bag bag;
    bag.open(rootPath + bagName, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.emplace_back("/rslidar_points");
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    if(recordBackground){
        std::cout << "Recording background..." << std::endl;
    }
    else{
        std::cout << "Skip record background..." << std::endl;
        std::cout << "### Using '-r' for background recording ###" << std::endl;
    }

    //define pointcloud publisher to rviz
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_info", 10);
    //define boundingbox publisher to rviz
    ros::Publisher bounding_boxes_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("bounding_boxes_info", 10);

    //define object publisher
    ros::Publisher objects_info_pub = nh.advertise<waytous_perception_msgs::ObjectArray>("/objects_info", 10);

    //indices for nan removing
    std::vector<int> indices;
    //Stream PCD
    auto* cloudIO = new LidarIO<pcl::PointXYZI>();
    //processing PCD
    auto* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();

    rosbag::View::iterator it = view.begin();

    //record background
    if (recordBackground) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr backgroundCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>);

        std::string backgroundDataPath = rootPath + "background";
        while ((backgroundNum > 0) & (it != view.end())){
            sensor_msgs::PointCloud2::ConstPtr input = (*it).instantiate<sensor_msgs::PointCloud2>();
            if (input != nullptr)
            {
                pcl::fromROSMsg(*input,*tempCloud);
                pcl::removeNaNFromPointCloud(*tempCloud, *tempCloud, indices);
                *backgroundCloud = *backgroundCloud + *tempCloud;
            }
            backgroundNum--;
            it ++;
        }
        //save background point cloud
        std::string outputPath = rootPath + "background.pcd";
        cloudIO->savePcd(backgroundCloud, outputPath);
    }

    //load background point cloud
    std::cout << "Loading background file..." << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr bgCloud(new pcl::PointCloud<pcl::PointXYZI>);
    bgCloud = cloudIO->loadPcd(rootPath + "background.pcd");
    // performing box filter and voxel filter on bgCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredBgCloud(new pcl::PointCloud<pcl::PointXYZI>);
    // box filter
    filteredBgCloud = pointProcessor->BoxFilter(bgCloud, minPoint, maxPoint);
    //voxel filter
    filteredBgCloud = pointProcessor->voxelFilter(filteredBgCloud, 0.3);

    //show video results
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZI>);
    while (ros::ok()){
        sensor_msgs::PointCloud2::ConstPtr input = (*it).instantiate<sensor_msgs::PointCloud2>();
        //publish point cloud
        point_cloud_pub.publish(input);

        //init object and bounding box message
        waytous_perception_msgs::ObjectArray lidar_detection_info;
        initPublisher(lidar_detection_info, 0);
        jsk_recognition_msgs::BoundingBoxArray bounding_boxes;

        if (input != nullptr) {
            pcl::fromROSMsg(*input, *inputCloud);
            pcl::removeNaNFromPointCloud(*inputCloud, *inputCloud, indices);
            //performing detection
            lidarDetection(pointProcessor, inputCloud, filteredBgCloud, lidar_detection_info);

            //copy header
            lidar_detection_info.header = input->header;
            bounding_boxes.header = input->header;

            //copy boundingbox info from lidar_detection_info to bounding_boxes
            copyBoxes(lidar_detection_info, bounding_boxes);

            // publish message
            objects_info_pub.publish(lidar_detection_info);
            //publish bounding boxes to rviz
            bounding_boxes_pub.publish(bounding_boxes);
        }
        it++;
        if(it == view.end()){
            it = view.begin();
        }
    }
}

void lidarDetection(ProcessPointClouds<pcl::PointXYZI>* pointProcessor,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredBgCloud,
                    waytous_perception_msgs::ObjectArray& lidar_detection_info)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredInputCloud(new pcl::PointCloud<pcl::PointXYZI>);
    // box filter
    filteredInputCloud = pointProcessor->BoxFilter(inputCloud, minPoint, maxPoint);
    //voxel filter
    filteredInputCloud = pointProcessor->voxelFilter(filteredInputCloud, 0.3);

    //remove background in inputCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr foregroundCloud(new pcl::PointCloud<pcl::PointXYZI>);
    foregroundCloud = pointProcessor->bkgRemove(filteredInputCloud, filteredBgCloud, 0.8, 1);
    //remove outlier
    // foregroundCloud = radiusFilter(foregroundCloud, 0.8, 5);

    // remove dust
    foregroundCloud = pointProcessor->dustRemove(foregroundCloud, 25.0, 100.0);

    if (foregroundCloud->points.empty()){
        return;
    }

    //Euclidean clustering
      // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->euclideanCluster(
      // foregroundCloud, 4, 3, 4000);

    //DBSCSN Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->DBSCANCluster(
            foregroundCloud, 2, 4, 3, 4000);

    if (cloudClusters.empty()){
        return;
    }

    int clusterId = 0;
    for(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        std::cout<<"ClusterId: " << clusterId<<std::endl;
        if(cluster->points.size() < 4){
            ++clusterId;
            continue;
        }
        //render box;
        BoxQ box = pointProcessor->minBoxQ(cluster);
        ++clusterId;

        //publish object info
        waytous_perception_msgs::Object object_info;
        generateObjectInfo(object_info, clusterId, cluster, box);
        lidar_detection_info.foreground_objects.push_back(object_info);
    }
}