#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/common.h>

#include "rslidar_sdk/src/msg/ros_msg/lidar_packet_ros.h"
#include "rslidar_sdk/src/msg/ros_msg/lidar_scan_ros.h"

#include <rs_driver/driver/decoder/decoder_factory.hpp>

using namespace robosense::lidar;


struct PointT
{
    float x;
    float y;
    float z;
    float intensity;
    double timestamp;
    int ring;
    uint8_t echotype;
};

int main(int argc, char* argv[])
{
    ros::init (argc, argv, "decoder_rs80");
    ros::NodeHandle nh;

    std::string projectRoot = "/home/guoxs/Documents/Project/Huituo/LidarDetectionRos/";
    std::string bagPath = projectRoot + "data/20201021/80/01.bag";
    std::string anglePath = projectRoot + "catkin_ws/src/rslidar_sdk/config/angle.csv";

    rosbag::Bag bag;
    bag.open(bagPath, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.emplace_back("/rslidar_packets");
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    rosbag::View::iterator it = view.begin();

    LidarType param_lidar_type = LidarType::RS80;
    RSDriverParam param;
    param.angle_path = anglePath;

    std::shared_ptr<DecoderBase<PointT>> decoder_rs80;
    decoder_rs80 = DecoderFactory<PointT>::createDecoder(param_lidar_type, param);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_2(new pcl::PointCloud<pcl::PointXYZI>);
    int height;

    ros::Publisher pcd_pub_1 = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_1", 1);
    ros::Publisher pcd_pub_2 = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_2", 1);

    while(ros::ok()){
        rslidar_msgs::rslidarScan::ConstPtr  rslidar_scan = (*it).instantiate<rslidar_msgs::rslidarScan>();
        rslidar_msgs::rslidarScan::_packets_type rs_packets = rslidar_scan->packets;
        for(const auto & p : rs_packets) {
            rslidar_msgs::rslidarPacket::_data_type rs_data = p.data;
            auto *pkt = (uint8_t *)rs_data.data();

            std::vector<PointT> pcd_temp;
            RSDecoderResult ret = decoder_rs80->processMsopPkt(pkt, pcd_temp, height);

            for (auto & point : pcd_temp){
                pcl::PointXYZI pt;
                pt.x = point.x;
                pt.y = point.y;
                pt.z = point.z;
                pt.intensity = point.intensity;

                if(point.echotype == 1)
                    pcd_1->points.push_back(pt);
                else if (point.echotype == 2)
                    pcd_2->points.push_back(pt);
                else
                    RS_ERROR<< "Unknow echo type: " << point.echotype << RS_REND;
            }

            if(ret == FRAME_SPLIT){
                sensor_msgs::PointCloud2 pcd_msg_1;
                sensor_msgs::PointCloud2 pcd_msg_2;

                pcl::toROSMsg(*pcd_1, pcd_msg_1);
                pcl::toROSMsg(*pcd_2, pcd_msg_2);

                pcd_msg_1.header.frame_id = "/rslidar";
                pcd_msg_2.header.frame_id = "/rslidar";

                pcd_pub_1.publish(pcd_msg_1);
                pcd_pub_2.publish(pcd_msg_2);

//                std::cout << "Point Cloud size of echo type 1: " << pcd_1->points.size() << std::endl;
//                std::cout << "Point Cloud size of echo type 2: " << pcd_2->points.size() << std::endl;

                pcd_1->clear();
                pcd_2->clear();
            }
        }
        it ++;
        if(it == view.end()){
            it = view.begin();
        }
    }
}