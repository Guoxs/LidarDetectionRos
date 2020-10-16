//
// Created by guoxs on 2020/10/14.
//

#include <vector>
#include <tf/tf.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_util.h"
#include <waytous_perception_msgs/ObjectArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>


void FromObjectMsg(const waytous_perception_msgs::ObjectArrayConstPtr& object_in,
                   vector<shared_ptr<Object>>& tracked_objects){
    for (auto &fore_object : object_in->foreground_objects)
    {
        std::shared_ptr<Object> temp_object(new Object);
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(fore_object.pointcloud,*temp_cloud);
        temp_object->cloud=temp_cloud;
        temp_object->type=ObjectType(fore_object.label_type);
        temp_object->center.x()=fore_object.pose.position.x;
        temp_object->center.y()=fore_object.pose.position.y;
        temp_object->center.z()=fore_object.pose.position.z;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(fore_object.pose.orientation,quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
        temp_object->direction.x()=cos(yaw);
        temp_object->direction.y()=sin(yaw);
        temp_object->direction.z()=0;
        temp_object->length=fore_object.dimensions.x;
        temp_object->width=fore_object.dimensions.y;
        temp_object->height=fore_object.dimensions.z;
        ConvertPolygon2Cloud(fore_object.convex_hull,temp_object->polygon);
        tracked_objects.push_back(temp_object);
    }
}

void ToObjectMsg(vector<shared_ptr<Object>>& objects_tracked,
                 waytous_perception_msgs::ObjectArray& tracked_objects_pub,
                 const waytous_perception_msgs::ObjectArrayConstPtr& object_in){
    tracked_objects_pub.header = object_in->header;
     for (auto & i : objects_tracked)
     {
         // msgs_zoo::DetectedObject temp_object;
         waytous_perception_msgs::Object temp_object;
         // temp_object.header=object_in->header;
         temp_object.id=i->id;
         // cout<<"current object id is"<<objects_tracked[i]->id_<<endl;
         sensor_msgs::PointCloud2 object_msg;
         pcl::toROSMsg(*(i->cloud), object_msg);
         object_msg.header = object_in->header;
         temp_object.pointcloud=object_msg;
         temp_object.label_type=i->type;
         temp_object.pose.position.x=i->center(0);
         temp_object.pose.position.y=i->center(1);
         temp_object.pose.position.z=i->center(2);
         temp_object.dimensions.x=i->length;
         temp_object.dimensions.y=i->width;
         temp_object.dimensions.z=i->height;
         //借用消息中的位置，赋值目标的线速度
         temp_object.velocity.x =i->velocity.head(2).norm();
         // temp_object.velocity.y=objects_tracked[i]->getAngle();
         temp_object.score=1.0;
         //将旋转角度转换成四元数
         tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, i->getAngle());
         tf::quaternionTFToMsg(quat, temp_object.pose.orientation);

         tracked_objects_pub.foreground_objects.push_back(temp_object);
     }
}

void copyBoxes(const waytous_perception_msgs::ObjectArray& tracked_objects_pub,
               jsk_recognition_msgs::BoundingBoxArray& bounding_boxes){
    for (const auto& object : tracked_objects_pub.foreground_objects){
        jsk_recognition_msgs::BoundingBox bbox;
        bbox.header = bounding_boxes.header;
        bbox.dimensions = object.dimensions;
        bbox.pose = object.pose;
        bbox.label = static_cast<char32_t>(object.label_type);
        bbox.value = object.id;
        bounding_boxes.boxes.push_back(bbox);
    }
}


