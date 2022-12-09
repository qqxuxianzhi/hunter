/******************************************************************************
 ** 消息接收模块
 ******************************************************************************
 *
 *  消息接收模块(从总线上接收报文)
 *
 *  @file       msg_receiver.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_MSG_RECEIVER_H_
#define PHOENIX_FRAMEWORK_MSG_RECEIVER_H_

#include "utils/macros.h"

#if (ENABLE_ROS_NODE)
#include <std_msgs/ByteMultiArray.h>

#include "communication/ros_node.h"
#endif

#if (ENABLE_LCM_NODE)
#include "communication/lcm_node.h"
#endif

#if (ENABLE_UDP_NODE)
#include "communication/udp_node.h"
#endif

#include "common/message.h"
#include "common/task.h"

namespace phoenix {
namespace framework {

class MsgReceiver : public Task {
 public:
  MsgReceiver(Task* manager);

#if (ENABLE_ROS_NODE)
  void SetRosNode(RosNode* node) { ros_node_ = node; }
#endif
#if (ENABLE_LCM_NODE)
  void SetLcmNode(LcmNode* node) { lcm_node_ = node; }
#endif
#if (ENABLE_UDP_NODE)
  void SetUdpNode(UdpNode* node) { udp_node_ = node; }
#endif

  bool Start();

 private:
#if (ENABLE_ROS_NODE)
  void HandleChassisMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleGnssMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleImuMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleLaserResultMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleRtkResultMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleRtkDataSourceMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandlePlanningStringMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleMqttMessageRos(const std_msgs::ByteMultiArray& msg);
#endif

#if (ENABLE_LCM_NODE)
#endif

 private:
#if (ENABLE_ROS_NODE)
  RosNode* ros_node_;
#endif
#if (ENABLE_LCM_NODE)
  LcmNode* lcm_node_;
#endif
#if (ENABLE_UDP_NODE)
  UdpNode* udp_node_;
#endif

#if (ENABLE_ROS_NODE)
  ad_msg::Chassis ros_chassis_info_;
  ad_msg::Gnss ros_gnss_info_;
  ad_msg::Imu ros_imu_info_;
  ad_msg::LaserResult ros_laser_result_;
  ad_msg::RtkResult ros_rtk_result_;
#endif
};

}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_MSG_RECEIVER_H_
