//
// Created by guoxs on 2020/9/23.
//
#include "IO/lidarIO.h"
#include "IO/lidarIO.cpp"
#include "render/render.h"
#include "processing/processPointClouds.h"
#include "processing/processPointClouds.cpp"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


//set point cloud range
const Eigen::Vector4f minPoint(0, -55, -5, 1);
const Eigen::Vector4f maxPoint( 100, 100, 15, 1);
//background frame number
int backgroundNum = 50;

void lidarDetection(pcl::visualization::PCLVisualizer::Ptr& viewer,
                    ProcessPointClouds<pcl::PointXYZI>* PointProcessor,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredBgCloud);

void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer);

void callBack(sensor_msgs::PointCloud2::ConstPtr data,
            pcl::visualization::PCLVisualizer::Ptr& viewer,
            ProcessPointClouds<pcl::PointXYZI>* pointProcessor,
            pcl::PointCloud<pcl::PointXYZI>::Ptr& backgroundCloud){
    //indices for nan removing
    std::vector<int> indices;
    if (data != nullptr)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::fromROSMsg(*data,*inputCloud);
        inputCloud->is_dense = false; //??? magic bug
        pcl::removeNaNFromPointCloud(*inputCloud, *inputCloud, indices);

        if (backgroundNum){
            std::cout << "Recording background..." << std::endl;
            *backgroundCloud = *backgroundCloud + *inputCloud;
            backgroundNum --;
            if  (backgroundNum == 0){
                std::cout << "Background recording finished, processing background file..." << std::endl;

                renderPointCloud(viewer, backgroundCloud, "backgroundCloud", Color(1,1,1));
                // box filter
                 backgroundCloud = pointProcessor->BoxFilter(backgroundCloud, minPoint, maxPoint);
                //voxel filter
                backgroundCloud = pointProcessor->voxelFilter(backgroundCloud, 0.3);
            }
//            renderPointCloud(viewer, backgroundCloud, "backgroundCloud"+backgroundNum, Color(1,1,1));
        }
        else{
            std::cout << "Lidar detection begin..." << std::endl;
            // Clear viewer
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            //default color is white
            renderPointCloud(viewer, inputCloud, "inputCloud", Color(0,1,1));
            //performing detection
            lidarDetection(viewer, pointProcessor, inputCloud, backgroundCloud);
            viewer->spinOnce ();
        }
        viewer->spinOnce ();
    }
}

void callBack2(sensor_msgs::PointCloud2::ConstPtr data,
              pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*data,*inputCloud);
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    renderPointCloud(viewer, inputCloud, "inputCloud", Color(1,1,1));
    viewer->spinOnce ();
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "lidar_detection_online");
    ros::NodeHandle n;
    std::string topicName = "/rslidar_points";

    auto* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr backgroundCloud(new pcl::PointCloud<pcl::PointXYZI>);
    // set viewer
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ros::Subscriber subscriber = n.subscribe<sensor_msgs::PointCloud2>(topicName, 5,
            boost::bind(&callBack, _1, viewer, pointProcessor, backgroundCloud));

//    ros::Subscriber subscriber = n.subscribe<sensor_msgs::PointCloud2>(
//            topicName, 5, boost::bind(&callBack, _1, viewer));
    ros::spin();
    return 0;
}

void lidarDetection(pcl::visualization::PCLVisualizer::Ptr& viewer,
                    ProcessPointClouds<pcl::PointXYZI>* pointProcessor,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& filteredBgCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredInputCloud(new pcl::PointCloud<pcl::PointXYZI>);
    // box filter
     filteredInputCloud = pointProcessor->BoxFilter(inputCloud, minPoint, maxPoint);
    //voxel filter
    filteredInputCloud = pointProcessor->voxelFilter(filteredInputCloud, 0.3);

    // renderPointCloud(viewer, filteredBgCloud,"filterBgCloud",Color(1,1,1));
    // renderPointCloud(viewer, filteredInputCloud,"filteredInputCloud",Color(1,1,1));

    //remove background in inputCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr foregroundCloud(new pcl::PointCloud<pcl::PointXYZI>);
    foregroundCloud = pointProcessor->bkgRemove(filteredInputCloud, filteredBgCloud, 0.8, 1);
    //remove outlier
    // foregroundCloud = radiusFilter(foregroundCloud, 0.8, 5);
    // renderPointCloud(viewer, foregroundCloud, "foregroundCloud",Color(1,0,0));

    // remove dust
    foregroundCloud = pointProcessor->dustRemove(foregroundCloud, 10.0, 1.5);

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

