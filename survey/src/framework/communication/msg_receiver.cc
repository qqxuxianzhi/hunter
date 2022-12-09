/******************************************************************************
 ** 消息接收模块
 ******************************************************************************
 *
 *  消息接收模块(从总线上接收报文)
 *
 *  @file       msg_receiver.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "communication/msg_receiver.h"

#include "communication/parse_proto_msg.h"
#include "data_serialization.h"
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
MsgReceiver::MsgReceiver(Task* manager)
    : Task(TASK_ID_MSG_RECEIVER, "Message Receiver", manager) {
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
/** 启动消息接收模块
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       启动消息接收模块，向总线订阅需要的报文
 *
 *  <Attention>
 *       None
 *
 */
bool MsgReceiver::Start() {
  bool ret = true;

#if (ENABLE_ROS_NODE)
  /// Comunicate by ROS
  if (Nullptr_t != ros_node_) {
    ret = ros_node_->Subscribe("control/chassis", 1,
                               &MsgReceiver::HandleChassisMessageRos, this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/chassis\".";
    }

    ret = ros_node_->Subscribe("localization/gnss", 1,
                               &MsgReceiver::HandleGnssMessageRos, this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/gnss\".";
    }

    ret = ros_node_->Subscribe("localization/imu", 1,
                               &MsgReceiver::HandleImuMessageRos, this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/imu\".";
    }

    ret = ros_node_->Subscribe("localization/laser_result", 1,
                               &MsgReceiver::HandleLaserResultMessageRos, this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/laser_result\".";
    }

    ret = ros_node_->Subscribe("localization/rtk_reuslt", 1,
                               &MsgReceiver::HandleRtkResultMessageRos, this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/rtk_reuslt\".";
    }

    ret =
        ros_node_->Subscribe("localization/rtk_data_source", 1,
                             &MsgReceiver::HandleRtkDataSourceMessageRos, this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/rtk_data_source\".";
    }

    ret = ros_node_->Subscribe("planning/string_message", 1,
                               &MsgReceiver::HandlePlanningStringMessageRos,
                               this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"planning/string_message\".";
    }

    ret = ros_node_->Subscribe("localization/mqtt", 1,
                               &MsgReceiver::HandleMqttMessageRos, this);
    if (false == ret) {
        LOG_ERR << "Failed to subscribe to \"localization/mqtt\".";
    }
  }
#endif  // #if (ENABLE_ROS_NODE)

#if (ENABLE_LCM_NODE)
  /// Comunicate by LCM
  if (Nullptr_t != lcm_node_) {
    // empty
  }
#endif  // #if (ENABLE_LCM_NODE)

#if (ENABLE_UDP_NODE)
  /// Comunicate by UDP
  if (Nullptr_t != udp_node_) {
    // empty
  }
#endif  // #if (ENABLE_UDP_NODE)

  return (ret);
}

#if (ENABLE_ROS_NODE)
void MsgReceiver::HandleChassisMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeChassisMessage((const Char_t*)(&msg.data[0]),
                                       msg.data.size(), &ros_chassis_info_)) {
      MessageChassis msg_chassis(&ros_chassis_info_);
      Notify(msg_chassis);
    }
  }
}

void MsgReceiver::HandleGnssMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }
  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeGnssMessage((const Char_t*)(&msg.data[0]),
                                    msg.data.size(), &ros_gnss_info_)) {
      MessageRecvGnssData msg_gnss(&ros_gnss_info_);
      Notify(msg_gnss);
    }
  }
}

void MsgReceiver::HandleImuMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }
  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeImuMessage((const Char_t*)(&msg.data[0]),
                                   msg.data.size(), &ros_imu_info_)) {
      MessageRecvImuData msg_imu(&ros_imu_info_);
      Notify(msg_imu);
    }
  }
}

void MsgReceiver::HandleLaserResultMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }
  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeLaserResultMessage((const Char_t*)(&msg.data[0]),
                                           msg.data.size(),
                                           &ros_laser_result_)) {
      MessageRecvLaserData msg_recv_laser_data(&ros_laser_result_);
      Notify(msg_recv_laser_data);
    }
  }
}

void MsgReceiver::HandleRtkResultMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }
  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeRtkResultMessage((const Char_t*)(&msg.data[0]),
                                         msg.data.size(), &ros_rtk_result_)) {
      MessageRecvRtkData msg_recv_rtk_data(&ros_rtk_result_);
      Notify(msg_recv_rtk_data);
    }
  }
}

void MsgReceiver::HandleRtkDataSourceMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    middle_layer_pb::RtkDataSource rtk_data;
    if (!rtk_data.ParseFromArray(&msg.data[0], msg.data.size())) {
      LOG_ERR << "Failed to parse imu from array.";
      return;
    }
    MessageRecvRtkDataSource msg_rtk(&rtk_data);
    Notify(msg_rtk);
  }
}

void MsgReceiver::HandlePlanningStringMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  // parse
  Char_t str[256];
  phoenix::common::com_memset(str, 0, sizeof(str));
  ;
  phoenix::common::com_memcpy(str, &msg.data[0],
                              MIN(msg.data.size(), sizeof(str)));
  std::string data(str);

  // set message data
  MessagePlanningString message(&data);

  // notify
  Notify(message);
}

void MsgReceiver::HandleMqttMessageRos(const std_msgs::ByteMultiArray &msg) {
    if (Nullptr_t == ros_node_) {
        return;
    }
    if (msg.data.size() > 0) {
        LOG_INFO(3) << "Receive Message from mqtt";
        // middle_layer_pb::RtkDataSource rtk_data;
        // if (!rtk_data.ParseFromArray(&msg.data[0], msg.data.size())) {
        //   LOG_ERR << "Failed to parse imu from array.";
        //   return;
        // }
        // MessageRecvRtkDataSource msg_rtk(&rtk_data);
        // Notify(msg_rtk);
    }
}

#endif  // #if (ENABLE_ROS_NODE)

}  // namespace framework
}  // namespace phoenix
