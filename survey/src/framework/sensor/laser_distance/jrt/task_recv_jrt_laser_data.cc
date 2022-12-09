#include "task_recv_jrt_laser_data.h"

#include <boost/algorithm/string.hpp>
#include <chrono>

#include "pc/util.h"
#include "serial_dev/posix/serial_driver_posix.h"
#include "utils/gps_tools.h"
namespace phoenix {
namespace sensor {
namespace laser {
TaskRecvJRTLaserData::TaskRecvJRTLaserData(framework::Task *manager)
    : framework::Task(framework::TASK_ID_RECV_JRT_LASER_DATA, "Recv Laser Data",
                      manager) {
  running_flag_recv_laser_ = false;
  is_laser_status_ = LASER_OFF;
  data_length_ = 0;
  common::com_memset(data_buff_, 0, sizeof(data_buff_));
  serial_dev_ = Nullptr_t;
  serial_dev_ = new serial_dev::SerialDriverPosix();
}

TaskRecvJRTLaserData::~TaskRecvJRTLaserData() {
  Stop();
  if (Nullptr_t != serial_dev_) {
    delete serial_dev_;
  }
}

bool TaskRecvJRTLaserData::Start() {
  if (running_flag_recv_laser_) {
    return (true);
  }

  data_length_ = 0;
  common::com_memset(data_buff_, 0, sizeof(data_buff_));
  if (Nullptr_t == serial_dev_) {
    LOG_ERR << "Invalid serial device.";
    return false;
  }
  serial_dev::SerialPortParam param;
  // com_snprintf(param.port_name, 64, "/dev/laser");
  com_snprintf(param.port_name, 64, "/dev/ttyTHS0");
  param.data_bits = serial_dev::SerialPortParam::DATA_BITS_8;
  param.parity = serial_dev::SerialPortParam::PARITY_NO;
  param.stop_bits = serial_dev::SerialPortParam::STOP_BITS_ONE;
  param.baud_rate = serial_dev::SerialPortParam::BAUD_RATE_19200;

  if (!serial_dev_->OpenPort(param)) {
    LOG_ERR << "Failed to open laser serial device.";
    return false;
  }
  LOG_INFO(3) << "Create thread of receiving laser data...";
  thread_recv_laser_ = boost::thread(
      boost::bind(&TaskRecvJRTLaserData::ThreadReceivingLaser, this));
  LOG_INFO(3) << "Create thread of receiving laser data... [OK]";
  running_flag_recv_laser_ = true;
  return (true);
}

bool TaskRecvJRTLaserData::Stop() {
  if (running_flag_recv_laser_) {
    running_flag_recv_laser_ = false;
    LOG_INFO(3) << "Stop thread of receiving laser data ...";
    bool ret = thread_recv_laser_.timed_join(boost::posix_time::seconds(1));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread of receiving laser data to stop.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop thread of receiving laser data ... [OK]";
    } else {
      LOG_INFO(3) << "Stop thread of receiving laser data ... [NG]";
    }

    CloseLaser();

    LOG_INFO(3) << "Close serial port.";
    if (Nullptr_t != serial_dev_) {
      serial_dev_->ClosePort();
    }
  }
  return (true);
}

void TaskRecvJRTLaserData::ThreadReceivingLaser() {
  LOG_INFO(3) << "Thread of receiving laser data ... [Started]";

  framework::Dormancy dormancy(50);
  Uint8_t data_buff[512];
  OpenLaser();
  while (running_flag_recv_laser_) {
    common::com_memset(data_buff, 0, sizeof(data_buff));
    Int32_t bytes = serial_dev_->ReadWait(data_buff, 510);
    if (bytes > 0) {
      std::cout << "bytes:" << bytes << std::endl;
      for (Int32_t i = 0; i < bytes; i++) {
        printf(" %02X", data_buff[i]);
      }
      std::cout << std::endl;
      SpinSerialData(&data_buff[0], bytes);
    }
    // dormancy.Sleep();
  }
  CntinusExit();
  LOG_INFO(3) << "Thread of receiving laser data ... [Stopped]";
}

void TaskRecvJRTLaserData::SpinSerialData(const Uint8_t *buffer,
                                          const Int32_t length) {
  if (length < 1) {
    return;
  }
  Int32_t msg_len = 0;
  for (Int32_t i = 0; i < length; ++i) {
    if (0 == data_length_) {
      /* Synch */
      if (0xAA == buffer[i]) {
        data_buff_[data_length_] = buffer[i];
        data_length_++;
      } else {
        data_length_ = 0;
      }
    } else if (1 == data_length_) {
      /* Synch */
      if (0x00 == buffer[i]) {
        data_buff_[data_length_] = buffer[i];
        data_length_++;
      } else {
        data_length_ = 0;
      }
    } else if (2 == data_length_) { /* Synch */
      if (0x01 == buffer[i] /*激光器开启*/ || 0x00 == buffer[i] /*测量结果*/) {
        data_buff_[data_length_] = buffer[i];
        data_length_++;
      } else {
        data_length_ = 0;
      }
    } else if (3 == data_length_) {
      if ((0xBE == buffer[i] /*激光器开启*/) ||
          (0x22 == buffer[i] /*测量结果*/)) {
        data_buff_[data_length_] = buffer[i];
        data_length_++;
      } else {
        data_length_ = 0;
      }
    } else {
      msg_len = 0;
      if (0xBE == data_buff_[3]) {
        msg_len = 9;
      }
      if (0x22 == data_buff_[3]) {
        msg_len = 13;
      }

      if (data_length_ < 8) {
        data_buff_[data_length_] = buffer[i];
        data_length_++;
      } else {
        if (data_length_ < (msg_len - 1)) {
          data_buff_[data_length_] = buffer[i];
          data_length_++;
        } else if (data_length_ == (msg_len - 1)) {
          data_buff_[data_length_] = buffer[i];
          data_length_++;
          // complete message frame
          ParseMessageFrame(data_buff_, data_length_);
          data_length_ = 0;
        } else {
          LOG_ERR << "Unexpected case occurs.";
          data_length_ = 0;
        }
      }
    }
  }
}

void TaskRecvJRTLaserData::ParseMessageFrame(const Uint8_t *buffer,
                                             Int32_t length) {
  //测量结果= 0xAABBCCDD毫米（帧字节6 = 0xAA， 字节7 =
  // 0xBB， 字节8 = 0xCC， 字节9 = 0xDD） ， 信号质量 =
  //     0x101， 信号质量数越小表示激光信号越强， 距 离结果越可靠
  // AA 00 00 22 00 03 00 00 04 75 00 18 B6
  // if (length == 9) {
  //   if (buffer[2] == 0x01 && buffer[3] == 0xBE) {
  //     if (buffer[7] == 0x01) {
  //       is_laser_status_ = LASER_ON;
  //     } else if (buffer[7] == 0x00) {
  //       is_laser_status_ = LASER_OFF;
  //     }
  //   }
  // } else
  if (length == 13) {
    Float64_t height = 0.0;
    Float64_t sq = 0.0;
    height =
        (buffer[6] << 24) + (buffer[7] << 16) + (buffer[8] << 8) + buffer[9];

    sq = (buffer[10] << 8) + buffer[11];
    LOG_INFO(3) << "height:" << height << " SQ:" << sq;
    height = height * 0.001;
    if (height > 0 && height <= 1) {
      phoenix::ad_msg::LaserResult laser_result;
      laser_result.msg_head.valid = true;
      laser_result.msg_head.timestamp = common::GetClockNowMs();
      laser_result.msg_head.UpdateSequenceNum();
      laser_result.height = height;
      phoenix::framework::MessageRecvLaserData message(&laser_result);
      Notify(message);
    } else {
      LOG_ERR << "LaserData Error:" << height;
    }
    is_laser_status_ = LASER_AUTO_CNT;
  }
}

void TaskRecvJRTLaserData::OpenLaser() {
  Uint8_t send_data_buff[9];
  common::com_memset(send_data_buff, 0, sizeof(send_data_buff));
  //开启激光器
  send_data_buff[0] = 0xAA;
  send_data_buff[1] = 0x00;
  send_data_buff[2] = 0x01;
  send_data_buff[3] = 0xBE;
  send_data_buff[4] = 0x00;
  send_data_buff[5] = 0x01;
  send_data_buff[6] = 0x00;
  send_data_buff[7] = 0x01;  // 0x01:激光开启, 0x00:激光关闭.
  send_data_buff[8] = 0xC1;
  serial_dev_->Send(send_data_buff, sizeof(send_data_buff));
  // is_laser_status_ = LASER_OPENING;
}

void TaskRecvJRTLaserData::CloseLaser() {
  Uint8_t send_data_buff[9];
  common::com_memset(send_data_buff, 0, sizeof(send_data_buff));
  //关闭激光器
  send_data_buff[0] = 0xAA;
  send_data_buff[1] = 0x00;
  send_data_buff[2] = 0x01;
  send_data_buff[3] = 0xBE;
  send_data_buff[4] = 0x00;
  send_data_buff[5] = 0x01;
  send_data_buff[6] = 0x00;
  send_data_buff[7] = 0x00;  // 0x01:激光开启, 0x00:激光关闭.
  send_data_buff[8] = 0xC0;
  serial_dev_->Send(send_data_buff, sizeof(send_data_buff));
  // is_laser_status_ = LASER_CLOSING;
}

//开启连续测量
void TaskRecvJRTLaserData::CntinusAuto() {
  Uint8_t send_data_buff[9];
  common::com_memset(send_data_buff, 0, sizeof(send_data_buff));
  //设置连续测量 AA 00 00 20 00 01 00 04 25
  send_data_buff[0] = 0xAA;
  send_data_buff[1] = 0x00;
  send_data_buff[2] = 0x00;
  send_data_buff[3] = 0x20;
  send_data_buff[4] = 0x00;
  send_data_buff[5] = 0x01;
  send_data_buff[6] = 0x00;
  send_data_buff[7] = 0x04;
  send_data_buff[8] = 0x25;
  serial_dev_->Send(send_data_buff, sizeof(send_data_buff));
  is_laser_status_ = LASER_AUTO_CNTING;
}
//关闭连续测量
void TaskRecvJRTLaserData::CntinusExit() {
  Uint8_t send_data_buff[1];
  common::com_memset(send_data_buff, 0, sizeof(send_data_buff));
  send_data_buff[0] = 0x58;
  serial_dev_->Send(send_data_buff, sizeof(send_data_buff));
  is_laser_status_ = LASER_EXIT_CNT;
}

//单次慢速测量
void TaskRecvJRTLaserData::OneShotSlow() {
  Uint8_t send_data_buff[9];
  common::com_memset(send_data_buff, 0, sizeof(send_data_buff));
  //设置单次慢速测量 AA 00 00 20 00 01 00 01 22
  send_data_buff[0] = 0xAA;
  send_data_buff[1] = 0x00;
  send_data_buff[2] = 0x00;
  send_data_buff[3] = 0x20;
  send_data_buff[4] = 0x00;
  send_data_buff[5] = 0x01;
  send_data_buff[6] = 0x00;
  send_data_buff[7] = 0x01;
  send_data_buff[8] = 0x22;
  serial_dev_->Send(send_data_buff, sizeof(send_data_buff));
}

Uint8_t TaskRecvJRTLaserData::GetLaserStatus() { return is_laser_status_; }

}  // namespace laser
}  // namespace sensor
}  // namespace phoenix
