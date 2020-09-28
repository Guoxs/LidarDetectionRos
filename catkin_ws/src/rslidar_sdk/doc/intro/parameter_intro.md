# Parameters Introduction

There is only one parameter file called config.yaml, which is locate in *rslidar_sdk/config/config.yaml*.  The whole file can be divided into two parts, the *common*  and *lidar* . In multi-LiDARs case, the parameters in *common* part will be shared by all LiDARs, while the parameters in *lidar* part need to be adjust according to specific LiDAR.  

**Note: The config.yaml is very strict to indentation! Please make sure the indentation is not changed when adjusting the parameters!**



## 1 Common

This part is used to decide the source of LiDAR data, and whether to send out the result or not.

```yaml
common:
  msg_source: 1                                         #0: not use Lidar
                                                        #1: packet message comes from online Lidar
                                                        #2: packet message comes from ROS or ROS2
                                                        #3: packet message comes from Pcap file
                                                        #4: packet message comes from Protobuf-UDP
                                                        #5: point cloud comes from Protobuf-UDP
  send_packet_ros: false                                #true: Send packets through ROS or ROS2(Used to record packet)
  send_point_cloud_ros: false                           #true: Send point cloud through ROS or ROS2
  send_packet_proto: false                              #true: Send packets through Protobuf-UDP
  send_point_cloud_proto: false                         #true: Send point cloud through Protobuf-UDP
  pcap_path: /home/robosense/lidar.pcap                 #The path of pcap file
```

- msg_source

  - 0 -- Not use lidar. Basically you will never set this parameter to 0.

  - 1 -- When connecting with a running lidar, set to 1. For more details, please refer to [Online connect lidar and send point cloud through ROS](../howto/how_to_online_send_point_cloud_ros.md)

  - 2 -- The lidar packet come from ROS or ROS2. This will be used in offline decode rosbag.  For more details, please refer to [Record rosbag & Offline decode rosbag](../howto/how_to_record_and_offline_decode_rosbag.md)

  - 3 -- The lidar packet come from offline pcap bag. For more details, please refer to  [Decode pcap bag and send point cloud through ROS](../howto/how_to_offline_decode_pcap.md)

  - 4 -- The lidar packet come from Protobuf-UDP. For more details, please refer to [Use protobuf send & receive](../howto/how_to_use_protobuf_function.md)

  - 5 -- The lidar point cloud come from Protobuf-UDP. For more details, please refer to  [Use protobuf send & receive](../howto/how_to_use_protobuf_function.md)



- send_packet_ros

      	True -- The lidar packets will be sent to ROS or ROS2. e.g. When you connect a lidar and want to record rosbag, you can set the *msg_source=1* and set *send_packet_ros = true*.
   
      ​	false -- Do nothing.
   
      ​ **Note1:  If the msg_source =2, there is no use to set send_packet_ros to true because the packet come from ROS and there is no reason to send them back to ROS.**
   
      ​	**Note2: Since the ROS packet message type is robosense self-defined type, you can't directly echo the topic through ROS. Mostly the packets are only used to record offline bag because the size is much smaller than point cloud.**

- send_point_cloud_ros

      	true -- The lidar point cloud will be sent to ROS or ROS2. e.g. When you connect a lidar and want to see point cloud on ROS-Rviz, you can the *msg_source =1* and set *send_point_cloud_ros = true*.
   
      ​	false -- Do nothing.
        
      ​	**Note: The ROS point cloud type is the ROS official defined type -- sensor_msgs/PointCloud2, which means the point cloud can be visualized on ROS-Rviz directly. Also you can record the point cloud to rosbag but its size may be very large, that's why we suggest to  record packets.**

- send_packet_proto

      	true -- The lidar packets will be sent out as protobuf message through ethernet in UDP protocal. e.g. When you connect the lidar with computerA and want to see the point cloud on computerB, you can run a rslidar_sdk on computerA and set the *msg_source = 1*, set *send_packet_proto = true*. Then, on computerB, set the *msg_source = 4* and set *send_point_cloud_ros = true*, then you can see the point cloud on computerB through ROS-Rviz.
   
      ​	false -- Do nothing

- send_point_cloud_proto

      	true -- The lidar point cloud will be sent out as protobuf message through ethernet in UDP protocal. e.g. When you connect the lidar with computerA and want to see the point cloud on computerB, you can run a rslidar_sdk on computerA and *set the msg_source = 1*, set *send_point_cloud_proto = true*. Then, on computerB, set the *msg_source = 5* and *set send_point_cloud_ros = true*, then you can see the point cloud on computerB through ROS-Rviz.
   
       **Node: We suggest send packets through ethernet rather than point cloud because point cloud size is too larger and it may take up a lot of bandwidth.**

- pcap_path

      	If the *msg_source = 3*, please make sure the pcap_path is correct. 



## 2 LiDAR

This part need to be adjust according to different LiDAR (in multi-LiDARs case). Here is an example for one LiDAR and three LiDARs. 



**One lidar example**

```yaml
lidar:
  - driver:
      lidar_type: RS128            #The lidar type - RS16, RS32, RSBP, RS128, RS80
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The msop port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the timestamp  
      angle_path: /home/robosense/angle.csv
    ros:
      ros_recv_packet_topic: /rslidar_packets    #The topic which used to receive lidar packets from ROS
      ros_send_packet_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_point_cloud_topic: /rslidar_points      #The topic which used to send point cloud through ROS
    proto:
      point_cloud_recv_port: 60021                     #The port number used for receiving point cloud 
      point_cloud_send_port: 60021                     #The port number which the point cloud will be send to
      point_cloud_send_ip: 127.0.0.1                   #The ip address which the point cloud will be send to 
      msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
      msop_send_port: 60022                       #The port number which the msop packets will be send to 
      difop_send_port: 60023                      #The port number which the difop packets will be send to 
      packet_send_ip: 127.0.0.1                   #The ip address which the lidar packets will be send to
```



**Three lidar example**

```yaml
lidar:
  - driver:
      lidar_type: RS128            #The lidar type - RS16, RS32, RSBP, RS128, RS80
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The msop port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the timestamp  
      angle_path: /home/robosense/angle.csv   #The path of the angle calibration file. For the latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packet_topic: /rslidar_packets    #The topic which used to receive lidar packets from ROS
      ros_send_packet_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_point_cloud_topic: /rslidar_points      #The topic which used to send point cloud through ROS
    proto:
      point_cloud_recv_port: 60021                     #The port number used for receiving point cloud 
      point_cloud_send_port: 60021                     #The port number which the point cloud will be send to
      point_cloud_send_ip: 127.0.0.1                   #The ip address which the point cloud will be send to 
      msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
      msop_send_port: 60022                       #The port number which the msop packets will be send to 
      difop_send_port: 60023                      #The port number which the difop packets will be send to 
      packet_send_ip: 127.0.0.1                   #The ip address which the lidar packets will be send to
  - driver:
      lidar_type: RSBP             #The lidar type - RS16, RS32, RSBP, RS128, RS80
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.199     #The device ip address
      msop_port: 1990              #The msop port of lidar,default is 6699
      difop_port: 1991             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the timestamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For the latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packet_topic: /left/rslidar_packets    #The topic which used to receive lidar packets from ROS
      ros_send_packet_topic: /left/rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_point_cloud_topic: /left/rslidar_points      #The topic which used to send point cloud through ROS
    proto:
      point_cloud_recv_port: 60024                     #The port number used for receiving point cloud 
      point_cloud_send_port: 60024                     #The port number which the point cloud will be send to
      point_cloud_send_ip: 127.0.0.1                   #The ip address which the point cloud will be send to 
      msop_recv_port: 60025                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60026                      #The port number used for receiving lidar difop packets
      msop_send_port: 60025                       #The port number which the msop packets will be send to 
      difop_send_port: 60026                      #The port number which the difop packets will be send to 
      packet_send_ip: 127.0.0.1                   #The ip address which the lidar packets will be send to
  - driver:
      lidar_type: RSBP            #The lidar type - RS16, RS32, RSBP, RS128, RS80
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.201     #The device ip address
      msop_port: 2000              #The msop port of lidar,default is 6699
      difop_port: 2001             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the timestamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For the latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packet_topic: /right/rslidar_packets    #The topic which used to receive lidar packets from ROS
      ros_send_packet_topic: /right/rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_point_cloud_topic: /right/rslidar_points      #The topic which used to send point cloud through ROS
    proto:
      point_cloud_recv_port: 60027                     #The port number used for receiving point cloud 
      point_cloud_send_port: 60027                     #The port number which the point cloud will be send to
      point_cloud_send_ip: 127.0.0.1                   #The ip address which the point cloud will be send to 
      msop_recv_port: 60028                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60029                      #The port number used for receiving lidar difop packets
      msop_send_port: 60028                       #The port number which the msop packets will be send to 
      difop_send_port: 60029                      #The port number which the difop packets will be send to 
      packet_send_ip: 127.0.0.1                   #The ip address which the lidar packets will be send to
```

