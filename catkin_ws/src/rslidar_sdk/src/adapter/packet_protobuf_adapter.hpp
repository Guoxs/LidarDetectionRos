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
#ifdef PROTO_FOUND
#define PKT_RECEIVE_BUF_SIZE 2000000
#include "adapter/adapter_base.h"
#include "utility/protobuf_communicator.hpp"
#include "msg/proto_msg_translator.h"
#include <condition_variable>
#include <mutex>

namespace robosense
{
namespace lidar
{
class PacketProtoAdapter : virtual public AdapterBase
{
public:
  PacketProtoAdapter() : old_frmnum_(0), new_frmnum_(0)
  {
    thread_pool_ptr_.reset(new ThreadPool());
  }

  ~PacketProtoAdapter()
  {
    stop();
  }

  void init(const YAML::Node& config)
  {
    bool send_packet_proto;
    int msg_source = 0;
    std::string packet_send_ip;
    std::string msop_send_port;
    std::string difop_send_port;
    uint16_t msop_recv_port;
    uint16_t difop_recv_port;
    yamlReadAbort<int>(config, "msg_source", msg_source);
    yamlRead<bool>(config, "send_packet_proto", send_packet_proto, false);
    yamlReadAbort<std::string>(config["proto"], "packet_send_ip", packet_send_ip);
    yamlReadAbort<std::string>(config["proto"], "msop_send_port", msop_send_port);
    yamlReadAbort<std::string>(config["proto"], "difop_send_port", difop_send_port);
    yamlReadAbort<uint16_t>(config["proto"], "msop_recv_port", msop_recv_port);
    yamlReadAbort<uint16_t>(config["proto"], "difop_recv_port", difop_recv_port);
    scan_proto_com_ptr_.reset(new ProtoCommunicator);
    packet_proto_com_ptr_.reset(new ProtoCommunicator);
    if (msg_source == MsgSource::MSG_FROM_PROTO_PACKET)
    {
      if ((scan_proto_com_ptr_->initReceiver(msop_recv_port) == -1) ||
          (packet_proto_com_ptr_->initReceiver(difop_recv_port) == -1))
      {
        RS_ERROR << "LidarPacketsReceiver: Create UDP Receiver Socket failed or Bind Network failed!" << RS_REND;
        exit(-1);
      }
      send_packet_proto = false;
    }
    if (send_packet_proto)
    {
      if ((scan_proto_com_ptr_->initSender(msop_send_port, packet_send_ip) == -1) ||
          (packet_proto_com_ptr_->initSender(difop_send_port, packet_send_ip) == -1))
      {
        RS_ERROR << "LidarPacketsReceiver: Create UDP Sender Socket failed ! " << RS_REND;
        exit(-1);
      }
    }
  }

  void start()
  {
    scan_buff_ = malloc(PKT_RECEIVE_BUF_SIZE);
    scan_recv_thread_.start_ = true;
    scan_recv_thread_.thread_.reset(new std::thread([this]() { recvMsopPkts(); }));
    packet_recv_thread_.start_ = true;
    packet_recv_thread_.thread_.reset(new std::thread([this]() { recvDifopPkts(); }));
  }

  void stop()
  {
    if (scan_recv_thread_.start_.load())
    {
      scan_recv_thread_.start_.store(false);
      scan_recv_thread_.thread_->join();
      free(scan_buff_);
    }
    if (packet_recv_thread_.start_.load())
    {
      packet_recv_thread_.start_.store(false);
      packet_recv_thread_.thread_->join();
    }
  }

  inline void regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
  {
    scan_cb_vec_.emplace_back(callback);
  }

  inline void regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
  {
    packet_cb_vec_.emplace_back(callback);
  }

  void sendScan(const ScanMsg& msg)
  {
    scan_send_queue_.push(msg);
    if (scan_send_queue_.is_task_finished_.load())
    {
      scan_send_queue_.is_task_finished_.store(false);
      thread_pool_ptr_->commit([this]() { sendMsop(); });
    }
  }

  void sendPacket(const PacketMsg& msg)
  {
    packet_send_queue_.push(msg);
    if (packet_send_queue_.is_task_finished_.load())
    {
      packet_send_queue_.is_task_finished_.store(false);
      thread_pool_ptr_->commit([this]() { sendDifop(); });
    }
  }

private:
  inline void localMsopCallback(const ScanMsg& rs_msg)
  {
    for (auto& cb : scan_cb_vec_)
    {
      cb(rs_msg);
    }
  }

  inline void localDifopCallback(const PacketMsg& rs_msg)
  {
    for (auto& cb : packet_cb_vec_)
    {
      cb(rs_msg);
    }
  }

private:
  void recvDifopPkts()
  {
    while (packet_recv_thread_.start_.load())
    {
      void* p_data = malloc(MAX_RECEIVE_LENGTH);
      ProtoMsgHeader tmp_header;
      int ret = packet_proto_com_ptr_->receiveProtoMsg(p_data, MAX_RECEIVE_LENGTH, tmp_header);

      if (ret == -1)
      {
        continue;
      }
      packet_recv_queue_.push(std::make_pair(p_data, tmp_header));
      if (packet_recv_queue_.is_task_finished_.load())
      {
        packet_recv_queue_.is_task_finished_.store(false);
        thread_pool_ptr_->commit([&]() { spliceDifopPkts(); });
      }
    }
  }

  void spliceDifopPkts()
  {
    while (packet_recv_queue_.size() > 0)
    {
      if (packet_recv_thread_.start_.load())
      {
        auto pair = packet_recv_queue_.front();
        proto_msg::LidarPacket protomsg;
        protomsg.ParseFromArray(pair.first, pair.second.msg_length);
        localDifopCallback(toRsMsg(protomsg));
      }
      free(packet_recv_queue_.front().first);
      packet_recv_queue_.pop();
    }
    packet_recv_queue_.is_task_finished_.store(true);
  }

  void recvMsopPkts()
  {
    bool start_check = true;
    while (scan_recv_thread_.start_.load())
    {
      void* p_data = malloc(MAX_RECEIVE_LENGTH);
      ProtoMsgHeader tmp_header;
      int ret = scan_proto_com_ptr_->receiveProtoMsg(p_data, MAX_RECEIVE_LENGTH, tmp_header);
      if (start_check)
      {
        if (tmp_header.msg_id == 0)
        {
          start_check = false;
        }
        else
        {
          continue;
        }
      }
      if (ret == -1)
      {
        RS_WARNING << "Packets Protobuf receiving error" << RS_REND;
        continue;
      }
      scan_recv_queue_.push(std::make_pair(p_data, tmp_header));
      if (scan_recv_queue_.is_task_finished_.load())
      {
        scan_recv_queue_.is_task_finished_.store(false);
        thread_pool_ptr_->commit([&]() { spliceMsopPkts(); });
      }
    }
  }

  void spliceMsopPkts()
  {
    while (scan_recv_queue_.size() > 0)
    {
      if (scan_recv_thread_.start_.load())
      {
        auto pair = scan_recv_queue_.front();
        old_frmnum_ = new_frmnum_;
        new_frmnum_ = pair.second.frame_num;
        memcpy((uint8_t*)scan_buff_ + pair.second.msg_id * SPLIT_SIZE, pair.first, SPLIT_SIZE);
        if ((old_frmnum_ == new_frmnum_) && (pair.second.msg_id == pair.second.total_msg_cnt - 1))
        {
          proto_msg::LidarScan proto_msg;
          proto_msg.ParseFromArray(scan_buff_, pair.second.total_msg_length);
          localMsopCallback(toRsMsg(proto_msg));
        }
      }
      free(scan_recv_queue_.front().first);
      scan_recv_queue_.pop();
    }
    scan_recv_queue_.is_task_finished_.store(true);
  }

  void sendDifop()
  {
    while (packet_send_queue_.size() > 0)
    {
      proto_msg::LidarPacket proto_msg = toProtoMsg(packet_send_queue_.popFront());
      if (!packet_proto_com_ptr_->sendSingleMsg<proto_msg::LidarPacket>(proto_msg))
      {
        RS_WARNING << "Difop packets Protobuf sending error" << RS_REND;
      }
    }
    packet_send_queue_.is_task_finished_.store(true);
  }

  void sendMsop()
  {
    while (scan_send_queue_.size() > 0)
    {
      proto_msg::LidarScan proto_msg = toProtoMsg(scan_send_queue_.popFront());
      if (!scan_proto_com_ptr_->sendSplitMsg<proto_msg::LidarScan>(proto_msg))
      {
        RS_WARNING << "Msop packets Protobuf sending error" << RS_REND;
      }
    }
    scan_send_queue_.is_task_finished_.store(true);
  }

private:
  std::vector<std::function<void(const ScanMsg&)>> scan_cb_vec_;
  std::vector<std::function<void(const PacketMsg&)>> packet_cb_vec_;
  std::unique_ptr<ProtoCommunicator> scan_proto_com_ptr_;
  std::unique_ptr<ProtoCommunicator> packet_proto_com_ptr_;
  lidar::ThreadPool::Ptr thread_pool_ptr_;
  lidar::Queue<ScanMsg> scan_send_queue_;
  lidar::Queue<PacketMsg> packet_send_queue_;
  lidar::Queue<std::pair<void*, ProtoMsgHeader>> scan_recv_queue_;
  lidar::Queue<std::pair<void*, ProtoMsgHeader>> packet_recv_queue_;
  lidar::Thread scan_recv_thread_;
  lidar::Thread packet_recv_thread_;
  int old_frmnum_;
  int new_frmnum_;
  void* scan_buff_;
};
}  // namespace lidar
}  // namespace robosense
#endif  // PROTO_FOUND