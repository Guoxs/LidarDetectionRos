//
// Created by guoxs on 2020/10/14.
//

#include <vector>
#include <tf/tf.h>
#include <pcl_conversions/pcl_conversions.h>

#include "../base/object.h"
#include "geometry_util.h"
#include <waytous_perception_msgs/ObjectArray.h>


void FromObjectMsg(const waytous_perception_msgs::ObjectArrayConstPtr& object_in,
                   vector<shared_ptr<Object>>& tracked_objects){
    for (size_t i = 0; i < object_in->foreground_objects.size(); i++)
    {
        std::shared_ptr<Object> temp_object(new Object);
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(object_in->foreground_objects[i].pointcloud,*temp_cloud);
        temp_object->cloud=temp_cloud;
        temp_object->type=ObjectType(object_in->foreground_objects[i].label_type);
        temp_object->center.x()=object_in->foreground_objects[i].pose.position.x;
        temp_object->center.y()=object_in->foreground_objects[i].pose.position.y;
        temp_object->center.z()=object_in->foreground_objects[i].pose.position.z;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(object_in->foreground_objects[i].pose.orientation,quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
        temp_object->direction.x()=cos(yaw);
        temp_object->direction.y()=sin(yaw);
        temp_object->direction.z()=0;
        temp_object->length=object_in->foreground_objects[i].dimensions.x;
        temp_object->width=object_in->foreground_objects[i].dimensions.y;
        temp_object->height=object_in->foreground_objects[i].dimensions.z;
        ConvertPolygon2Cloud(object_in->foreground_objects[i].convex_hull,temp_object->polygon);
        tracked_objects[i]=temp_object;
    }
}

void ToObjectMsg(vector<shared_ptr<Object>>& objects_tracked,
                 waytous_perception_msgs::ObjectArray tracked_objects_pub,
                 const waytous_perception_msgs::ObjectArrayConstPtr& object_in){
    tracked_objects_pub.header = object_in->header;
     for (int i = 0; i < objects_tracked.size(); ++i)
     {
         // msgs_zoo::DetectedObject temp_object;
         waytous_perception_msgs::Object temp_object;
         // temp_object.header=object_in->header;
         temp_object.id=objects_tracked[i]->id;
         // cout<<"current object id is"<<objects_tracked[i]->id_<<endl;
         sensor_msgs::PointCloud2 object_msg;
         pcl::toROSMsg(*(objects_tracked[i]->cloud), object_msg);
         object_msg.header = object_in->header;
         temp_object.pointcloud=object_msg;
         temp_object.label_type=objects_tracked[i]->type;
         temp_object.pose.position.x=objects_tracked[i]->center(0);
         temp_object.pose.position.y=objects_tracked[i]->center(1);
         temp_object.pose.position.z=objects_tracked[i]->center(2);
         temp_object.dimensions.x=objects_tracked[i]->length;
         temp_object.dimensions.y=objects_tracked[i]->width;
         temp_object.dimensions.z=objects_tracked[i]->height;
         //借用消息中的位置，赋值目标的线速度
         temp_object.velocity.x =objects_tracked[i]->velocity.head(2).norm();
         // temp_object.velocity.y=objects_tracked[i]->getAngle();
         temp_object.score=1.0;
         //将旋转角度转换成四元数
         tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, objects_tracked[i]->getAngle());
         tf::quaternionTFToMsg(quat, temp_object.pose.orientation);

         tracked_objects_pub.foreground_objects.push_back(temp_object);
     }
}

