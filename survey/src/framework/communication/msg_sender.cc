/******************************************************************************
 ** 消息发送模块
 ******************************************************************************
 *
 *  消息发送模块(发送报文到总线上)
 *
 *  @file       msg_sender.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "communication/msg_sender.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#if (ENABLE_ROS_NODE)
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/String.h"
#endif

#include "communication/parse_proto_msg.h"
#include "data_serialization.h"
#include "math/math_utils.h"
#include "utils/com_utils.h"

namespace phoenix {
namespace framework {

/******************************************************************************/
/** 构造函数
 ******************************************************************************
 *  @param      manager         (IN)           任务管理模块
 *              ros_node        (IN)           ros节点模块
 *
 *  @retval     None
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       构造函数
 *
 *  <Attention>
 *       None
 *
 */
MsgSender::MsgSender(Task* manager)
    : Task(TASK_ID_MSG_SENDER, "Message Sender", manager) {
#if (ENABLE_ROS_NODE)
  ros_node_ = Nullptr_t;
#endif

#if (ENABLE_LCM_NODE)
  lcm_node_ = Nullptr_t;
#endif

#if (ENABLE_UDP_NODE)
  udp_node_ = Nullptr_t;
#endif
}

/******************************************************************************/
/** 启动消息发送模块
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       启动消息发送模块，注册到总线上
 *
 *  <Attention>
 *       None
 *
 */
bool MsgSender::Start() {
  bool ret = true;

#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>("localization/gnss",
                                                            10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"localization/gnss\".";
    }
    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>("localization/imu",
                                                            10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"localization/imu\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
        "localization/rtk_reuslt", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"localization/rtk_reuslt\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
        "localization/rtk_data_source", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"localization/rtk_data_source\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
        "survey/string_message", 1);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 " \"survey/string_message\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::String>(
        "control/mqtt", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"control/mqtt\".";
    }


  }
#endif

  return (ret);
}

void MsgSender::SendGnssData(const ad_msg::Gnss gnss) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::localization::Gnss message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeGnssMessage(gnss, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>("localization/gnss",
                                                        msg)) {
        LOG_ERR << "Failed to send GNSS message.";
      }
    } else {
      LOG_ERR << "Failed to serialize GNSS message.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::localization::Gnss message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeGnssMessage(gnss, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish("localization/gnss", data_buff, serialize_size) <
          0) {
        LOG_ERR << "Failed to send GNSS message.";
      }
    } else {
      LOG_ERR << "Failed to serialize GNSS message.";
    }
    delete[] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(ad_msg::Gnss);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = data_serial::EncodeGnssArray(
          serialization_data_buff_, 0, max_buff_size, &gnss, 1);
      udp_node_->Publish("localization/gnss", serialization_data_buff_,
                         data_len);
    }
  }
#endif
}

void MsgSender::SendImuData(const ad_msg::Imu imu) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::localization::Imu message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeImuMessage(imu, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>("localization/imu",
                                                        msg)) {
        LOG_ERR << "Failed to send IMU message.";
      }
    } else {
      LOG_ERR << "Failed to serialize IMU message.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::localization::Imu message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeImuMessage(imu, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish("localization/imu", data_buff, serialize_size) <
          0) {
        LOG_ERR << "Failed to send IMU message.";
      }
    } else {
      LOG_ERR << "Failed to serialize IMU message.";
    }
    delete[] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(ad_msg::Imu);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = data_serial::EncodeImuArray(serialization_data_buff_,
                                                     0, max_buff_size, &imu, 1);
      udp_node_->Publish("localization/imu", serialization_data_buff_,
                         data_len);
    }
  }
#endif
}

void MsgSender::SendLaserResult(const ad_msg::LaserResult laser) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    middle_layer_pb::LaserResult message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeLaserResultMessage(laser, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
              "localization/laser_result", msg)) {
        LOG_ERR << "Failed to send LaserResult message.";
      }
    } else {
      LOG_ERR << "Failed to serialize LaserResult message.";
    }
  }
#endif
}

void MsgSender::SendRtkResult(const ad_msg::RtkResult rtk) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    middle_layer_pb::RtkResult message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeRtkResultMessage(rtk, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
              "localization/rtk_reuslt", msg)) {
        LOG_ERR << "Failed to send GNSS message.";
      }
    } else {
      LOG_ERR << "Failed to serialize GNSS message.";
    }
  }
#endif
}

void MsgSender::SendRtkDataSource(
    const middle_layer_pb::RtkDataSource message) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
              "localization/rtk_data_source", msg)) {
        LOG_ERR << "Failed to send Rtk Data Source message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Rtk Data Source message.";
    }
  }
#endif
}

void MsgSender::SendStringMsg(const std::string& msg) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    Int32_t serialize_size = msg.size();
    std_msgs::ByteMultiArray byte_multi_array;
    byte_multi_array.data.resize(serialize_size);
    common::com_memcpy(&byte_multi_array.data[0], msg.c_str(), serialize_size);
    if (!ros_node_->Publish<std_msgs::ByteMultiArray>("survey/string_message",
                                                      byte_multi_array)) {
      LOG_ERR << "Failed to send message with topic"
                 " \"survey/string_message\".";
    }
  }
#endif
}

 void MsgSender::SendWritingProcessInfo(const ad_msg::WritingWordInfo writing_info){
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    std::string str;
    ParseProtoMsg parse_msg;
    if(parse_msg.EncodeWritingProcessInfoMessage(writing_info, str)){
        std_msgs::String msg;
        msg.data = str.c_str();
        //std::cout << msg.data << std::endl;
        if (!ros_node_->Publish<std_msgs::String>("control/mqtt",
                                                          msg)) {
          LOG_ERR << "Failed to send message with topic"
                    " \"control/mqtt\".";
        }
    }
  }
#endif
 }


}  // namespace framework
}  // namespace phoenix
