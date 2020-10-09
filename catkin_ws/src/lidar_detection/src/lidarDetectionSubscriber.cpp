//
// Created by guoxs on 2020/10/9.
//
#include <ros/ros.h>
#include "waytous_perception_msgs/ObjectArray.h"

void objectsInfoCallback(const waytous_perception_msgs::ObjectArray::ConstPtr& msg)
{
    ROS_INFO("Subcribe objects Info: scence_type:%d", msg->current_scene.type);
    for (const auto& object : msg->foreground_objects){
        ROS_INFO("Subcribe object Info: obj_id:%d obj_label:%d obj_rect:{%2f, %2f, %2f, %2f}  "
                 "obj_ori:%2f obj_dimensions:{w: %2f, l:%2f, h:%2f}",
                 object.id, object.label_type, object.rect.x, object.rect.y, object.rect.w, object.rect.h,
                 object.orientation, object.dimensions.x, object.dimensions.y, object.dimensions.z);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_detection_subscriber");

    ros::NodeHandle nh;

    ros::Subscriber objects_info_pub = nh.subscribe("/objects_info", 5, objectsInfoCallback);

    ros::spin();

    return 0;
}
