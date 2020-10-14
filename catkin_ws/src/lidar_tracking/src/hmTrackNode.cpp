/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-15 12:37:02
 */

#include <vector>
#include <cstdio>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "base/object.h"
#include "filter/object_track.h"
#include "filter/hm_tracker.h"
#include "common/msg_utils.cpp"
#include <waytous_perception_msgs/ObjectArray.h>


ros::Publisher pub_track_array;
string range_objects_topic,tracked_objects_topic;

std::shared_ptr<HmObjectTracker> hm_tracker;
Eigen::Matrix4f transform_lidar = Eigen::Matrix4f::Identity();

vector<double> times;


void callback(const waytous_perception_msgs::ObjectArrayConstPtr& object_in)
{
    /////计时开始
    // clock_t startTime, endTime;
    // startTime = std::clock();
    // ROS_INFO_STREAM("*************node hm_track callback successed!!!**************");
    waytous_perception_msgs::ObjectArray observation;
    double timestamp = object_in->header.stamp.toSec();
    vector<shared_ptr<Object>> tracked_objects;
    vector<shared_ptr<Object>> objects_tracked;

    FromObjectMsg(object_in, tracked_objects);

    hm_tracker->Track(tracked_objects,timestamp,transform_lidar,&objects_tracked);

    waytous_perception_msgs::ObjectArray tracked_objects_pub;
    ToObjectMsg(objects_tracked, tracked_objects_pub, object_in);

    pub_track_array.publish(tracked_objects_pub);
    
    // endTime = std::clock(); //计时结束
    // times.push_back((double)(endTime - startTime) / CLOCKS_PER_SEC);
    // double mean = accumulate(begin(times), end(times), 0.0)/times.size();
    // ROS_INFO_STREAM("\033[0mThe hm_track run time is: " << mean << "s" );
    // ROS_INFO_STREAM("****************node hm_track end!!!************************");
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "hm_track_node");
    ros::NodeHandle nh;
    hm_tracker.reset(new HmObjectTracker);
    hm_tracker->Init(nh);

    ros::param::get("~detected_objects_topic",range_objects_topic);
    ros::param::get("~tracked_objects_topic",tracked_objects_topic);

    ROS_INFO_STREAM("[hm_track_node]订阅话题："<<range_objects_topic);
    ROS_INFO_STREAM("[hm_track_node]发布话题："<<tracked_objects_topic);
    
    ros::Subscriber sub_pcl = nh.subscribe<waytous_perception_msgs::ObjectArray>(range_objects_topic, 5, callback);
    pub_track_array = nh.advertise<waytous_perception_msgs::ObjectArray>(tracked_objects_topic, 5);

    ros::spin();
    return 0;
}