//
// Created by guoxs on 2020/9/23.
//
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "msg_util.cpp"
#include "IO/lidarIO.h"
#include "IO/lidarIO.cpp"
#include "render/render.h"
#include "processing/processPointClouds.h"
#include "processing/processPointClouds.cpp"


void lidarDetection(pcl::visualization::PCLVisualizer::Ptr& viewer,
                    ProcessPointClouds<pcl::PointXYZI>* PointProcessor,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredBgCloud,
                    waytous_perception_msgs::ObjectArray& lidar_detection_info);

void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer);

int main (int argc, char** argv)
{
    ros::init (argc, argv, "lidar_detection_bag");
    ros::NodeHandle nh;

    std::string bagRoot = argv[1];
    std::string bagName = argv[2];
    std::string rootPath = "/home/guoxs/Documents/Project/Huituo/LidarDetectionRos/data/";

    std::cout << "Starting Detecting..." << std::endl;

    rosbag::Bag bag;
    bag.open(rootPath + bagRoot + bagName + ".bag", rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.emplace_back("/rslidar_points");
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    //define object publisher
    ros::Publisher objects_info_pub = nh.advertise<waytous_perception_msgs::ObjectArray>("/objects_info", 5);
    waytous_perception_msgs::ObjectArray lidar_detection_info;
    // type: normal
    initPublisher(lidar_detection_info, 0);

    //indices for nan removing
    std::vector<int> indices;
    //Stream PCD
    auto* cloudIO = new LidarIO<pcl::PointXYZI>();
    //processing PCD
    auto* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();

    rosbag::View::iterator it = view.begin();

    //load background point cloud
    std::cout << "Loading background file..." << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr bgCloud(new pcl::PointCloud<pcl::PointXYZI>);
    bgCloud = cloudIO->loadPcd(rootPath + bagRoot + "background.pcd");

    // set viewer
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    //show video results
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZI>);
    while (!viewer->wasStopped ()){
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        sensor_msgs::PointCloud2::ConstPtr input = (*it).instantiate<sensor_msgs::PointCloud2>();
        if (input != nullptr)
        {
            pcl::fromROSMsg(*input,*inputCloud);
            inputCloud->is_dense = false;
            pcl::removeNaNFromPointCloud(*inputCloud, *inputCloud, indices);
            //default color is white
            renderPointCloud(viewer,inputCloud,"inputCloud", Color(1,1,1));

            //performing detection
            lidarDetection(viewer, pointProcessor, inputCloud, bgCloud, lidar_detection_info);
        }

        // publish message
        objects_info_pub.publish(lidar_detection_info);

        it++;
        if(it == view.end()){
            it = view.begin();
        }
        viewer->spinOnce ();
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
}

void lidarDetection(pcl::visualization::PCLVisualizer::Ptr& viewer,
                    ProcessPointClouds<pcl::PointXYZI>* pointProcessor,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredBgCloud,
                    waytous_perception_msgs::ObjectArray& lidar_detection_info)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredInputCloud(new pcl::PointCloud<pcl::PointXYZI>);
    //voxel filter
    filteredInputCloud = pointProcessor->voxelFilter(inputCloud, 0.3);

    // renderPointCloud(viewer, filteredBgCloud,"filterBgCloud",Color(1,1,1));
    // renderPointCloud(viewer, filteredInputCloud,"filteredInputCloud",Color(1,1,1));

    //remove background in inputCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr foregroundCloud(new pcl::PointCloud<pcl::PointXYZI>);
    foregroundCloud = pointProcessor->bkgRemove(filteredInputCloud, filteredBgCloud, 0.9, 1);
    //remove outlier
//     foregroundCloud = pointProcessor->radiusFilter(foregroundCloud, 2.5, 3);
    // renderPointCloud(viewer, foregroundCloud, "foregroundCloud",Color(1,0,0));

    // remove dust
    foregroundCloud = pointProcessor->dustRemove(foregroundCloud, 25.0, 100.0);

    if (foregroundCloud->points.empty()){
        return;
    }

    //Euclidean clustering
//    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->euclideanCluster(
//       foregroundCloud, 2.5, 3, 3000);

    //DBSCSN Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->DBSCANCluster(
            foregroundCloud, 5, 2.5, 5, 3000);

    if (cloudClusters.empty()){
        return;
    }

    // renderPointCloud(viewer,cloudClusters[0],"obstCloud",Color(1,0,0));
    // std::cout<<"cloud size: "<< cloudClusters.size() << std::endl;

    int clusterId = 0;
    std::vector<Color> colors = {
            Color(1,0,0),
            Color(0,1,0),
            Color(0,0,1)};

    for(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"foreCloud"+std::to_string(clusterId), colors[clusterId]);
        std::cout<<"ClusterId: " << clusterId<<std::endl;
        if(cluster->points.size() < 4){
            ++clusterId;
            continue;
        }
        //render box;
        BoxQ box = pointProcessor->minBoxQ(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;

        //publish object info
        waytous_perception_msgs::Object object_info;
        generateObjectInfo(object_info, clusterId, cluster, box);
        lidar_detection_info.foreground_objects.push_back(object_info);
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }
    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

