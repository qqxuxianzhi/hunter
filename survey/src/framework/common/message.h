/******************************************************************************
 ** 消息定义
 ******************************************************************************
 *
 *  定义各种用于通讯的消息类型
 *
 *  @file       message.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_MESSAGE_H_
#define PHOENIX_FRAMEWORK_MESSAGE_H_

#include <memory>
#include <vector>

#include "ad_msg.h"
#include "common/module_status.h"
#include "middle_layer.pb.h"
#include "utils/com_utils.h"
#include "utils/macros.h"

namespace phoenix {
namespace framework {

enum MessageId {
    MSG_ID_INVALID = 0,
    MSG_ID_REQ_DO_TASK,
    MSG_ID_RECV_IMU_DATA,
    MSG_ID_RECV_GNSS_DATA,
    MSG_ID_CHASSIS,
    MSG_ID_LASER_DATA,
    MSG_ID_RECV_LASER_DATA,
    MSG_ID_RECV_LI_LASER_DATA,
    MSG_ID_RTK_DATA,
    MSG_ID_RECV_RTK_DATA,
    MSG_ID_RTK_DATA_SOURCE,
    MSG_ID_RECV_RTK_DATA_SOURCE,
    MSG_ID_PLANNING_STRING
};

enum TaskId {
    TASK_ID_INVALID = 0,
    TASK_ID_MANAGER,
    TASK_ID_MSG_RECEIVER,
    TASK_ID_MSG_SENDER,
    TASK_ID_MSG_SERVER,
    TASK_ID_RECV_IMU_DATA,
    TASK_ID_RECV_GNSS_DATA,
    TASK_ID_RECV_NMEA_DATA,
    TASK_ID_RECV_LASER_DATA,
    TASK_ID_RECV_LI_LASER_DATA,
    TASK_ID_RECV_JRT_LASER_DATA,
    TASK_ID_RECV_RTK_DATA,
    TASK_ID_SERVO_CONTROL
};

class Message {
 public:
  explicit Message(int id) : id_(id) {}
  virtual ~Message() = default;

  Message(const Message& other) { id_ = other.id_; }

  void operator=(const Message& other) { id_ = other.id_; }

  int id() const { return (id_); }

 private:
  int id_;
};

class MessageModuleStatus : public Message {
 public:
  explicit MessageModuleStatus(int id) : Message(id) {
    status_ = ad_msg::MODULE_STATUS_OK;
    common::com_memset(param_, 0, sizeof(param_));
  }

  void set_status(int status) { status_ = status; }
  int status() const { return (status_); }

  void set_param0(int param) { param_[0] = param; }
  int param0() const { return (param_[0]); }
  void set_param1(int param) { param_[1] = param; }
  int param1() const { return (param_[1]); }

 private:
  int status_;
  int param_[4];
};

class MessageRecvImuData : public Message {
 public:
  MessageRecvImuData(const ad_msg::Imu* imu)
      : Message(MSG_ID_RECV_IMU_DATA), imu_(imu) {}

  const ad_msg::Imu* imu() const { return (imu_); }

 private:
  const ad_msg::Imu* imu_ = Nullptr_t;
};

class MessageRecvGnssData : public Message {
 public:
  MessageRecvGnssData(const ad_msg::Gnss* gnss)
      : Message(MSG_ID_RECV_GNSS_DATA), gnss_(gnss) {}
  const ad_msg::Gnss* gnss() const { return (gnss_); }

 private:
  const ad_msg::Gnss* gnss_ = Nullptr_t;
};

class MessageChassis : public Message {
 public:
  MessageChassis(const ad_msg::Chassis* data)
      : Message(MSG_ID_CHASSIS), chassis_(data) {}
  inline const ad_msg::Chassis* chassis() const { return (chassis_); }

 private:
  const ad_msg::Chassis* chassis_ = Nullptr_t;
};

class MessageLaserData : public Message {
 public:
  MessageLaserData(const ad_msg::LaserResult* data)
      : Message(MSG_ID_LASER_DATA), laser_result_(data) {}

  inline const ad_msg::LaserResult* laserResult() const {
    return (laser_result_);
  }

 private:
  const ad_msg::LaserResult* laser_result_ = Nullptr_t;
};

class MessageRecvLaserData : public Message {
 public:
  MessageRecvLaserData(const ad_msg::LaserResult* data)
      : Message(MSG_ID_RECV_LASER_DATA), laser_result_(data) {}

  inline const ad_msg::LaserResult* laserResult() const {
    return (laser_result_);
  }

 private:
  const ad_msg::LaserResult* laser_result_ = Nullptr_t;
};

class MessageRecvLiLaserData : public Message {
  public:
    MessageRecvLiLaserData(const ad_msg::LaserLiResult *data)
        : Message(MSG_ID_RECV_LI_LASER_DATA), laser_result_(data) {}

    inline const ad_msg::LaserLiResult *laserLiResult() const {
        return (laser_result_);
    }

  private:
    const ad_msg::LaserLiResult *laser_result_ = Nullptr_t;
};

class MessageRtkData : public Message {
 public:
  MessageRtkData(const ad_msg::RtkResult* data)
      : Message(MSG_ID_RTK_DATA), rtk_result_(data) {}
  inline const ad_msg::RtkResult* rtkResult() const { return (rtk_result_); }

 private:
  const ad_msg::RtkResult* rtk_result_ = Nullptr_t;
};

class MessageRecvRtkData : public Message {
 public:
  MessageRecvRtkData(const ad_msg::RtkResult* data)
      : Message(MSG_ID_RECV_RTK_DATA), rtk_result_(data) {}
  inline const ad_msg::RtkResult* rtkResult() const { return (rtk_result_); }

 private:
  const ad_msg::RtkResult* rtk_result_ = Nullptr_t;
};

class MessageRtkDataSource : public Message {
 public:
  MessageRtkDataSource(const middle_layer_pb::RtkDataSource* data)
      : Message(MSG_ID_RTK_DATA_SOURCE), rtk_data_source_(data) {}
  inline const middle_layer_pb::RtkDataSource* rtk_data_source() const {
    return (rtk_data_source_);
  }

 private:
  const middle_layer_pb::RtkDataSource* rtk_data_source_ = Nullptr_t;
};

class MessageRecvRtkDataSource : public Message {
 public:
  MessageRecvRtkDataSource(const middle_layer_pb::RtkDataSource* data)
      : Message(MSG_ID_RECV_RTK_DATA_SOURCE), rtk_data_source_(data) {}
  inline const middle_layer_pb::RtkDataSource* rtk_data_source() const {
    return (rtk_data_source_);
  }

 private:
  const middle_layer_pb::RtkDataSource* rtk_data_source_ = Nullptr_t;
};

class MessagePlanningString : public Message {
 public:
  explicit MessagePlanningString(const std::string* data)
      : Message(MSG_ID_PLANNING_STRING) {
    planning_string_ = data;
  }

  const std::string* planning_string() const { return planning_string_; }

 private:
  const std::string* planning_string_;
};

}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_MESSAGE_H_
