//
// Created by guoxs on 2020/10/9.
//
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Polygon.h>
#include <pcl_conversions/pcl_conversions.h>

#include "render/box.h"
#include "global.h"
#include "waytous_perception_msgs/ObjectArray.h"

void static initPublisher(waytous_perception_msgs::ObjectArray lidar_detection_info, int type) {
    // type: normal
    lidar_detection_info.current_scene.type = type;
    waytous_perception_msgs::Rect scence_rect;
    scence_rect.x = minPoint[0];
    scence_rect.y = minPoint[1];
    scence_rect.w = maxPoint[0] - minPoint[0];
    scence_rect.h = maxPoint[1] - minPoint[1];
    lidar_detection_info.current_scene.rect = {scence_rect};
    lidar_detection_info.background_objects = {};
    lidar_detection_info.current_scene.reliable = {1.0};
}

void static generateObjectInfo(waytous_perception_msgs::Object object_info, int obj_id,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud, BoxQ box) {
    object_info.id = obj_id;
    // sensor type: lidar
    object_info.sensor_type = 1;
    //label type: trunk
    object_info.label_type = 3;
    // 2D Rect
    waytous_perception_msgs::Rect rect_2d;
    rect_2d.x = box.vertex3[0];
    rect_2d.y = box.vertex3[1];
    rect_2d.h = box.cube_length;
    rect_2d.w = box.cube_width;
    object_info.rect = rect_2d;
    object_info.orientation = atan2(box.direction[0], box.direction[1]);
    //3D Dimension
    //pose
    geometry_msgs::Pose pose;
    pose.position.x = box.bboxTransform[0];
    pose.position.y = box.bboxTransform[1];
    pose.position.z = box.bboxTransform[2];
    pose.orientation.x = box.bboxQuaternion.x();
    pose.orientation.y = box.bboxQuaternion.y();
    pose.orientation.z = box.bboxQuaternion.z();
    pose.orientation.w = box.bboxQuaternion.w();
    object_info.pose = pose;
    //dimensions
    object_info.dimensions.x = box.cube_width;
    object_info.dimensions.y = box.cube_length;
    object_info.dimensions.z = box.cube_height;
    //pointcloud
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(*pointcloud, cloud);
    object_info.pointcloud = cloud;
    //Polygon
    object_info.convex_hull = {};

    object_info.score = 1.0;

    object_info.track_id = obj_id;

    object_info.velocity.x = 0.0;
    object_info.velocity.y = 0.0;
    object_info.velocity.z = 0.0;

    object_info.velocity_confidence = 1.0;
    object_info.velocity_covariance = {};
    object_info.motion_state = 1;
    object_info.trace = {};
}
