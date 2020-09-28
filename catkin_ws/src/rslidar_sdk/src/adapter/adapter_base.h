/******************************************************************************
 * Copyright 2020 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#pragma once
#include "utility/common.h"
#include "utility/yaml_reader.hpp"
#include "rs_driver/msg/packet_msg.h"
#include "rs_driver/msg/scan_msg.h"
#include "msg/rs_msg/lidar_point_cloud_msg.h"

namespace robosense
{
namespace lidar
{
enum MsgSource
{
  MSG_FROM_LIDAR = 1,
  MSG_FROM_ROS_PACKET = 2,
  MSG_FROM_PCAP = 3,
  MSG_FROM_PROTO_PACKET = 4,
  MSG_FROM_PROTO_POINTCLOUD = 5
};

class AdapterBase
{
public:
  typedef std::shared_ptr<AdapterBase> Ptr;
  AdapterBase() = default;
  virtual ~AdapterBase() = default;

  virtual void init(const YAML::Node& config) = 0;

  virtual void start()
  {
    return;
  }

  virtual void stop()
  {
    return;
  }

  virtual void sendScan(const ScanMsg& msg)
  {
    return;
  }

  virtual void sendPacket(const PacketMsg& msg)
  {
    return;
  }

  virtual void sendPointCloud(const LidarPointCloudMsg& msg)
  {
    return;
  }

  virtual void sendCameraTrigger(const CameraTrigger& msg)
  {
    return;
  }

  virtual void regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
  {
    return;
  }

  virtual void regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
  {
    return;
  }

  virtual void regRecvCallback(const std::function<void(const LidarPointCloudMsg&)>& callback)
  {
    return;
  }

  virtual void regRecvCallback(const std::function<void(const CameraTrigger&)>& callback)
  {
    return;
  }

  virtual void decodeScan(const ScanMsg& msg)
  {
    return;
  }

  virtual void decodePacket(const PacketMsg& msg)
  {
    return;
  }
};
}  // namespace lidar
}  // namespace robosense
