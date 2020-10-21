#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "rslidar_sdk/src/msg/ros_msg/lidar_packet_ros.h"
#include "rslidar_sdk/src/msg/ros_msg/lidar_scan_ros.h"
#include "rs_driver/driver/decoder/decoder_RS80.hpp"

using namespace robosense::lidar;

struct PointXYZI
{
    float x;
    float y;
    float z;
    float intensity;
};

int main(int argc, char* argv[])
{
    ros::init (argc, argv, "decoder_rs80");
    ros::NodeHandle nh;

    std::string bagPath = "/home/guoxs/Documents/Project/Huituo/LidarDetectionRos/data/echo_test/test.bag";

    rosbag::Bag bag;
    bag.open(bagPath, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.emplace_back("/rslidar_packets");
    // topics.emplace_back("/rslidar_packets_difop");
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    rosbag::View::iterator it = view.begin();

    RSDecoderParam param;
    LidarConstantParameter lidar_param;
    lidar_param.LASER_NUM = 80;
    DecoderRS80<PointXYZI> decoder(param, lidar_param);

    while(it != view.end()){
        // rslidar_msgs::rslidarPacket::ConstPtr rslidar_pck = (*it).instantiate<rslidar_msgs::rslidarPacket>();
        rslidar_msgs::rslidarScan::ConstPtr  rslidar_scan = (*it).instantiate<rslidar_msgs::rslidarScan>();
        rslidar_msgs::rslidarScan::_packets_type rs_packets = rslidar_scan->packets;
        for(const auto & p : rs_packets){
            rslidar_msgs::rslidarPacket::_stamp_type rs_stamp = p.stamp;
            rslidar_msgs::rslidarPacket::_data_type  rs_data = p.data;

            std::vector<PointXYZI> vec;
            int height = 0;
            int azimuth = 0;
            // decoder.decodeMsopPkt(*rs_data, vec, height, azimuth);





            boost::array<uint8_t, 80> data_header{};
            boost::array<uint8_t, 976> data_body{};
            boost::array<uint8_t, 192> data_tail{};

            for (int i = 0; i < rslidar_msgs::rslidarPacket::_data_type::size(); ++i) {

                if (i < 80)
                    data_header[i] = rs_data[i];
                else if (i < 1056)
                    data_body[i-80] = rs_data[i];
                else
                    data_tail[i-1056] = rs_data[i];
            }
            std::cout << data_body.size() << std::endl;
        }

        it ++;
    }


}
