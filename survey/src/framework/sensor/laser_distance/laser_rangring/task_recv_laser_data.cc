#include "task_recv_laser_data.h"

#include <boost/algorithm/string.hpp>
#include <chrono>

#include "pc/util.h"
#include "serial_dev/posix/serial_driver_posix.h"
#include "utils/gps_tools.h"
namespace phoenix {
namespace sensor {
namespace laser {
TaskRecvLaserData::TaskRecvLaserData(framework::Task *manager)
    : framework::Task(framework::TASK_ID_RECV_LASER_DATA, "Recv Laser Data",
                      manager) {
  running_flag_recv_laser_ = false;
  common::com_memset(data_buff_, 0, sizeof(data_buff_));

  serial_dev_ = Nullptr_t;
  serial_dev_ = new serial_dev::SerialDriverPosix();
}

TaskRecvLaserData::~TaskRecvLaserData() {
  Stop();
  if (Nullptr_t != serial_dev_) {
    delete serial_dev_;
  }
}

bool TaskRecvLaserData::Start() {
  if (running_flag_recv_laser_) {
    return (true);
  }

  common::com_memset(data_buff_, 0, sizeof(data_buff_));
  if (Nullptr_t == serial_dev_) {
    LOG_ERR << "Invalid serial device.";
    return false;
  }
  serial_dev::SerialPortParam param;
  com_snprintf(param.port_name, 64, "/dev/laser");
  param.data_bits = serial_dev::SerialPortParam::DATA_BITS_8;
  param.parity = serial_dev::SerialPortParam::PARITY_NO;
  param.stop_bits = serial_dev::SerialPortParam::STOP_BITS_ONE;
  param.baud_rate = serial_dev::SerialPortParam::BAUD_RATE_9600;

  if (!serial_dev_->OpenPort(param)) {
    LOG_ERR << "Failed to open laser serial device.";
    return false;
  }

  LOG_INFO(3) << "Create thread of receiving laser data...";
  thread_recv_laser_ = boost::thread(
      boost::bind(&TaskRecvLaserData::ThreadReceivingLaser, this));
  LOG_INFO(3) << "Create thread of receiving laser data... [OK]";
  running_flag_recv_laser_ = true;

  return (true);
}

bool TaskRecvLaserData::Stop() {
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

    Uint8_t send_data_buff[4];
    common::com_memset(send_data_buff, 0, sizeof(send_data_buff));
    //关机
    send_data_buff[0] = 0x80;
    send_data_buff[1] = 0x04;
    send_data_buff[2] = 0x02;
    send_data_buff[3] = 0x7A;
    serial_dev_->Send(send_data_buff, sizeof(send_data_buff));

    LOG_INFO(3) << "Close serial port.";
    if (Nullptr_t != serial_dev_) {
      serial_dev_->ClosePort();
    }
  }
  return (true);
}

void TaskRecvLaserData::ThreadReceivingLaser() {
  LOG_INFO(3) << "Thread of receiving laser data ... [Started]";
  OpenLaser();  //开启激光器

  int isStarted = 0;
  framework::Dormancy dormancy(50);
  dormancy.Sleep();
  dormancy.Sleep();
  SetFreq();
  dormancy.Sleep();
  dormancy.Sleep();
  StartLaserRanging();
  dormancy.Sleep();
  dormancy.Sleep();

  while (running_flag_recv_laser_) {
    common::com_memset(data_buff_, 0, sizeof(data_buff_));
    Int32_t bytes = serial_dev_->ReadWait(data_buff_, 510);
    if (bytes > 0) {
      if (data_buff_[2] == 0x85 && data_buff_[3] == 0x01) {
        StartLaserRanging();
      } else {
        Float64_t height = SpinSerialData(&data_buff_[0], bytes);

        if (height > 0 && height <= 1) {
          phoenix::ad_msg::LaserResult laser_result;
          laser_result.msg_head.valid = true;
          laser_result.msg_head.timestamp = common::GetClockNowMs();
          laser_result.msg_head.UpdateSequenceNum();
          laser_result.height = height;
          phoenix::framework::MessageRecvLaserData message(&laser_result);
          Notify(message);
        }
      }
    }
  }
  CloseLaser();
  LOG_INFO(3) << "Thread of receiving laser data ... [Stopped]";
}

Float64_t TaskRecvLaserData::SpinSerialData(const Uint8_t *buffer,
                                            const Int32_t length) {
  //在单次测量和连续测量返回数据中,引号中为数据部分,其格式为 ASCII 格式
  //如:  123.456 米 显示为 31 32 33 2E 34 35 36
  Float64_t height = 0.0;
  if (length >= 11) {
    height = (buffer[3] - 0x30) * 100 + (buffer[4] - 0x30) * 10 +
             (buffer[5] - 0x30) + (buffer[7] - 0x30) * 0.1 +
             (buffer[8] - 0x30) * 0.01 + (buffer[9] - 0x30) * 0.001;
    // LOG_INFO(3) << "height:" << height;
    height = 0.45;
  }
  if (height > 1.0) {
    LOG_ERR << "LaserData Error:" << height;
  }
  return height;
}

void TaskRecvLaserData::OpenLaser() {
  Uint8_t send_data_buff[5];
  common::com_memset(send_data_buff, 0, sizeof(send_data_buff));
  //开启激光器 80 06 05 01 74
  send_data_buff[0] = 0x80;
  send_data_buff[1] = 0x06;
  send_data_buff[2] = 0x05;
  send_data_buff[3] = 0x01;
  send_data_buff[4] = 0x74;
  serial_dev_->Send(send_data_buff, sizeof(send_data_buff));
}
void TaskRecvLaserData::CloseLaser() {
  Uint8_t send_data_buff[5];
  common::com_memset(send_data_buff, 0, sizeof(send_data_buff));
  //关闭激光器 80 06 05 01 74
  send_data_buff[0] = 0x80;
  send_data_buff[1] = 0x06;
  send_data_buff[2] = 0x05;
  send_data_buff[3] = 0x00;
  send_data_buff[4] = 0x75;
  serial_dev_->Send(send_data_buff, sizeof(send_data_buff));
}
void TaskRecvLaserData::StartLaserRanging() {
  Uint8_t send_data_buff[4];
  common::com_memset(send_data_buff, 0, sizeof(send_data_buff));
  //设置连续测量 80 06 03 77
  send_data_buff[0] = 0x80;
  send_data_buff[1] = 0x06;
  send_data_buff[2] = 0x03;
  send_data_buff[3] = 0x77;
  serial_dev_->Send(send_data_buff, sizeof(send_data_buff));
}
// FA 04 0A 0A EE
void TaskRecvLaserData::SetFreq() {
  Uint8_t send_data_buff[5];
  common::com_memset(send_data_buff, 0, sizeof(send_data_buff));
  // 频率 10Hz FA 04 0A 0A EE
  // 20Hz  FA 04 0A 14 E4
  send_data_buff[0] = 0xFA;
  send_data_buff[1] = 0x04;
  send_data_buff[2] = 0x0A;
  send_data_buff[3] = 0x14;
  send_data_buff[4] = 0xE4;
  serial_dev_->Send(send_data_buff, sizeof(send_data_buff));
}
}  // namespace laser
}  // namespace sensor
}  // namespace phoenix
