# **rslidar_sdk**

## 1 工程简介
 **rslidar_sdk** 为速腾聚创在Ubuntu环境下的雷达驱动软件包，包括了雷达驱动内核， ROS拓展功能，ROS2拓展功能，Protobuf-UDP通信拓展功能。对于没有二次开发需求的用户，或是想直接使用ROS或ROS2进行二次开发的用户，可直接使用本软件包， 配合ROS或ROS2自带的RVIZ可视化工具即可查看点云。 对于有二次开发需求，想将雷达驱动集成到自己工程内的客户， 请参考雷达驱动内核的相关文档，直接使用内核[rs_driver](https://github.com/RoboSense-LiDAR/rs_driver)进行二次开发。



## 2 下载

### 2.1 使用git clone

 由于rslidar_sdk项目中包含子模块驱动内核rs_driver, 因此在执行git clone 后还需要执行相关指令初始化并更新子模块。

  ```sh
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update
  ```

### 2.2 直接下载

由于直接下载的压缩包内不包含git信息，因此需要手动下载驱动内核[rs_driver](https://github.com/RoboSense-LiDAR/rs_driver),  然后将其解压在 rslidar_sdk/src路径下。 将src中原有的rs_driver文件夹删除，再将刚刚解压好的rs_driver-xxxx文件夹改名为rs_driver即可。





## 3 依赖介绍

### 3.1 ROS 

若需在ROS环境下使用雷达驱动，则需安装ROS相关依赖库。

Ubuntu 16.04 - 安装ROS kinetic desktop-full  

Ubuntu 18.04 - 安装ROS melodic desktop-full

安装方式： 参考 http://wiki.ros.org

**如果安装了ROS kinetic desktop-full版或ROS melodic desktop-full版，那么兼容版本其他依赖库也应该同时被安装了，所以不需要重新安装它们以避免多个版本冲突引起的问题, 因此，强烈建议安装desktop-full版，这将节省大量的时间来逐个安装和配置库**。

### 3.2 ROS2

若需在ROS2环境下使用雷达驱动，则需安装ROS2相关依赖库。

Ubuntu 16.04 - 不支持

Ubuntu 18.04 - 安装ROS2 Eloquent desktop

安装方式：参考 https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/

**注意！ 请避免在同一台电脑上同时安装ROS和ROS2， 这可能会产生冲突！ 同时还需要手动安装Yaml库**。

### 3.3 Yaml (必需)

版本号:  >= v0.5.2 

若已安装ROS desktop-full, 可跳过。

安装方式:

```sh
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev
```

### 3.4 Pcap (必需)

版本号： >=v1.7.4

安装方式：

```sh
sudo apt-get install -y  libpcap-dev
```

### 3.5 Protobuf (可选)

版本号： >=v2.6.1

安装方式:

```sh
sudo apt-get install -y libprotobuf-dev protobuf-compiler
```



## 4 编译 & 运行

我们提供三种编译&运行方式。

### 4.1 直接编译

(1) 按照如下指令即可编译运行程序。 直接编译也可以使用ROS相关功能(不包括ROS2)，但需要在程序启动前**手动启动roscore**，启动后**手动打开rviz**才能看到可视化点云结果。

```sh
cd rslidar_sdk
mkdir build && cd build
cmake .. && make -j4
./rslidar_sdk_node
```

### 4.2 依赖于ROS-catkin编译

(1) 打开工程内的*CMakeLists.txt*文件，将文件顶部的**set(COMPILE_METHOD ORIGINAL)**改为**set(COMPILE_METHOD CATKIN)**。

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)
```

(2) 将rslidar_sdk工程目录下的*package_ros1.xml*文件重命名为*package.xml*。

(3) 新建一个文件夹作为工作空间，然后再新建一个名为*src*的文件夹, 将rslidar_sdk工程放入*src*文件夹内。

(4) 返回工作空间目录，执行以下命令即可编译&运行(若使用.zsh,将第二句指令替换为 *source devel/setup.zsh*)。

```sh
catkin_make
source devel/setup.bash
roslaunch rslidar_sdk start.launch
```

### 4.3 依赖于ROS2-colcon编译

(1) 打开工程内的*CMakeLists.txt*文件，将文件顶部的**set(COMPILE_METHOD ORIGINAL)**改为**set(COMPILE_METHOD COLCON)**。

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD COLCON)
```

(2) 将rslidar_sdk工程目录下的*package_ros2.xml*文件重命名为*package.xml*。

(3) 新建一个文件夹作为工作空间，然后再新建一个名为*src*的文件夹, 将rslidar_sdk工程放入*src*文件夹内。

(4) 通过[链接](https://github.com/RoboSense-LiDAR/rslidar_msg)下载ROS2环境下的雷达Packet消息定义， 将rslidar_msg工程也放在刚刚新建的*src*文件夹内，与rslidar_sdk并列。

(5) 返回工作空间目录，执行以下命令即可编译&运行(若使用.zsh,将第二句指令替换为 *source install/setup.zsh*)。

```sh
colcon build
source install/setup.bash
ros2 launch rslidar_sdk start.py
```



## 5 文件结构

```sh
.						
├── config        #存放所有的参数文件
├── doc           #存放所有的文档
│   ├── howto       #存放使用文档
│   └── intro       #存放介绍文档
├── launch        #存放ROS与ROS2的启动脚本
├── node          #存放节点代码（main函数)
├── rviz          #存放ROS与ROS2的rviz配置文件
└── src           #存放所有源代码
    ├── adapter     #存放外围适配器的代码
    ├── manager     #存放适配器管理器的代码
    ├── msg         #存放消息定义
    ├── rs_driver   #驱动内核
    └── utility     #存放工具类代码
```



## 6 参数介绍

**参数介绍非常重要，请仔细阅读。 本软件包的所有功能都将通过配置参数文件来实现。**

[参数介绍](doc/intro/parameter_intro.md)



## 7 快速上手

**以下仅为一些常用功能的快速使用指南， 实际使用时并不仅限于以下几种工作模式， 可通过配置参数改变不同的工作模式。**

[在线读取雷达数据发送到ROS](doc/howto/how_to_online_send_point_cloud_ros_cn.md)

[录制ROS数据包&离线解析ROS数据包](doc/howto/how_to_record_and_offline_decode_rosbag_cn.md)

[离线解析Pcap包发送到ROS](doc/howto/how_to_offline_decode_pcap_cn.md)



## 8 使用进阶

[隐藏参数介绍](doc/intro/hiding_parameters_intro.md)

[使用Protobuf发送&接收](doc/howto/how_to_use_protobuf_function_cn.md)

[多雷达](doc/howto/how_to_use_multi_lidars_cn.md)





---



## 1 Introduction

**rslidar_sdk** is the lidar driver software development kit under Ubuntu operating system, which contains the lidar driver core, ROS support, ROS2 support and Protobuf-UDP communication functions. For user who want to get point cloud through ROS or ROS2,  this software development kit can be used directly. For user who want to do advanced development or integrate the lidar driver into their own projects, please refer to the lidar driver core [rs_driver](https://github.com/RoboSense-LiDAR/rs_driver).



## 2 Download

### 2.1 Use git clone

Since rslidar_sdk project include the submodule --- rs_driver, user need to excute the following commands after git clone.

```sh
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update
```

### 2.2 Download directly

Since the zip file does not include submodule information, user need to download the driver core [rs_driver](https://github.com/RoboSense-LiDAR/rs_driver). Then unzip the rs_driver under the path */rslidar_sdk/src*. Delete the original empty *rs_driver* folder, and rename the folder you just unzip *rs_driver-XXXX* to *rs_driver*.  



## 3 Dependencies

### 3.1 ROS

If use rslidar_sdk in ROS environment, ROS related libraries need to be installed. 

Ubuntu 16.04 - Install ROS kinetic desktop-full

Ubuntu 18.04 - install ROS melodic desktop-full

Installation： please refer to  http://wiki.ros.org

**If you install ROS kinetic desktop-full or ROS melodic desktop-full，then the correspond PCL and Boost  will be installed at the same time. It will bring you a lot of convenience since you don't need to handle the version confliction. Thus, its highly recommanded to install ROS  desktop-full**.

### 3.2 ROS2

If use rslidar_sdk in ROS2 environment, ROS2 related libraries need to be installed. 

Ubuntu 16.04 - Not support 

Ubuntu 18.04 - Install ROS2 eloquent desktop

Installation: please refer to https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/

**Note! Please avoid to install ROS and ROS2 in one computer at the same time! This may cause confliction! Also you may need to install Yaml  manually.**

### 3.3 Yaml(Essential) 

version: >= v0.5.2

If installed ROS desktop-full, this part can be ignored.

Installation:

```sh
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev
```

### 3.4 Pcap(Essential) 

version: >=v1.7.4

Installation：

```sh
sudo apt-get install -y  libpcap-dev
```

### 3.5 Protobuf(Optional)

version:>=v2.6.1

Installation :

```sh
sudo apt-get install -y libprotobuf-dev protobuf-compiler
```


## 4 Compile & Run

We offer three ways to compile and run the driver.

### 4.1 Compile directly

(1) Excute the commands below. In this way, user can also use ROS functions(Not include ROS2), but need to start **roscore** manually before running the driver and need to start **rviz** manually to watch the point cloud.

```sh
cd rslidar_sdk
mkdir build && cd build
cmake .. && make -j4
./rslidar_sdk_node
```

### 4.2 Compile with ROS-catkin

(1) Open the *CMakeLists.txt* in the project，modify the line  on top of the file **set(COMPILE_METHOD ORIGINAL)** to **set(COMPILE_METHOD CATKIN)**.

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)
```

(2) Rename the file *package_ros1.xml*  in the rslidar_sdk to *package.xml*

(3) Create a new folder as the workspace, and create a *src* folder in the workspace. Then put the rslidar_sdk project in the *src* folder.

(4) Return back to the workspace, excute the following command to compile and run. (if use .zsh, replace the 2nd command with *source devel/setup.zsh*).

```sh
catkin_make
source devel/setup.bash
roslaunch rslidar_sdk start.launch
```

### 4.3 Compile with ROS2-colcon

(1) Open the *CMakeLists.txt* in the project，modify the line  on top of the file **set(COMPILE_METHOD ORIGINAL)** to **set(COMPILE_METHOD COLCON)**.

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD COLCON)
```

(2) Rename the file *package_ros2.xml*  in the rslidar_sdk to *package.xml*

(3) Create a new folder as the workspace, and create a *src* folder in the workspace. Then put the rslidar_sdk project in the *src* folder.

(4) Download the packet definition project in ROS2 through [link](https://github.com/RoboSense-LiDAR/rslidar_msg), then put the project rslidar_msg in the *src* folder you just created.

(5) Return back to the workspace, excute the following command to compile and run. (if use .zsh, replace the 2nd command with *source install/setup.zsh*).

```sh
colcon build
source install/setup.bash
ros2 launch rslidar_sdk start.py
```



### 5 File Structure

```sh
.						
├── config        #Store all the config files
├── doc           #Store all the documents
│   ├── howto       #Store the guide documents
│   └── intro       #Store the introduction documents
├── launch        #Store the launch files for ROS & ROS2
├── node          #Store the node code(main function)
├── rviz          #Store the rviz config file for ROS & ROS2
└── src           #Store all the source codes
    ├── adapter     #Store the code of adapters
    ├── manager     #Store the code of adapter manager
    ├── msg         #Store the message definition
    ├── rs_driver   #The lidar driver core
    └── utility     #Store the tool codes
```



### 6 Introduction to parameters

**This part is very important so please read carefully. All the functions of this software development kit will be reach by modifying parameters.**

[Intro to parameters](doc/intro/parameter_intro.md)



### 7 Quick start

**The following documents are some quick guides for using some of the most common features of the rslidar_sdk.  The rslidar_sdk are not limited to the following modes of operation and user can use rslidar_sdk in their own way by modifying parameters.**

[Online connect lidar and send point cloud through ROS](doc/howto/how_to_online_send_point_cloud_ros.md)

[Record rosbag & Offline decode rosbag](doc/howto/how_to_record_and_offline_decode_rosbag.md)

[Decode pcap bag and send point cloud through ROS](doc/howto/how_to_offline_decode_pcap.md)



### 8 Advanced

[Intro to hiding parameters](doc/intro/hiding_parameters_intro.md)

[Use protobuf send & receive](doc/howto/how_to_use_protobuf_function.md)

[Multi-LiDARs](doc/howto/how_to_use_multi_lidars.md)
