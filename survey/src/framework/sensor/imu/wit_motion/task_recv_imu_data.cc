//
#include "sensor/imu/wit_motion/task_recv_imu_data.h"

#include "can_dev/zlg_can_net/can_driver_zlgcannet.h"
#include "communication/shared_data.h"
#include "utils/com_utils.h"
#include "utils/log.h"

#define ENABLE_IMU_MSG_TRACE (0)

namespace phoenix {
namespace sensor {
namespace imu {
namespace wit_motion {
TaskRecvImuData::TaskRecvImuData(framework::Task *manager)
    : framework::Task(framework::TASK_ID_RECV_IMU_DATA, "Recv Imu Data",
                      manager) {
  running_flag_recv_imu_ = false;

  can_channel_ = Nullptr_t;
  can_channel_ = new can_dev::CanDriverZlgCanNet();
}

TaskRecvImuData::~TaskRecvImuData() {
  Stop();

  if (Nullptr_t != can_channel_) {
    delete can_channel_;
  }
}

bool TaskRecvImuData::Start() {
  if (Nullptr_t == can_channel_) {
    LOG_ERR << "Invalid Can channel.";
    // return false;
  }

  if (running_flag_recv_imu_) {
    return (true);
  }

  can_dev::CanChannelParam can_channel_param;
  can_channel_param.channel = 0;
  can_channel_param.bit_rate = can_dev::CAN_BIT_RATE_250K;
  com_snprintf(can_channel_param.can_net.ip_addr, 30, "192.168.1.178");
  can_channel_param.can_net.port = 4001;

  if (!can_channel_->OpenChannel(can_channel_param)) {
    LOG_ERR << "Falied to open CAN channel " << can_channel_param.channel;
    // return false;
  }

  LOG_INFO(3) << "Create thread of receiving imu data...";
  running_flag_recv_imu_ = true;
  thread_recv_imu_ =
      boost::thread(boost::bind(&TaskRecvImuData::ThreadReceivingImu, this));

  LOG_INFO(3) << "Create thread of receiving imu data... [OK]";

  return (true);
}

bool TaskRecvImuData::Stop() {
  if (running_flag_recv_imu_) {
    running_flag_recv_imu_ = false;

    LOG_INFO(3) << "Stop thread of receiving imu data ...";
    bool ret = thread_recv_imu_.timed_join(boost::posix_time::seconds(1));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread of receiving imu data to stop.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop thread of receiving imu data ... [OK]";
    } else {
      LOG_INFO(3) << "Stop thread of receiving imu data ... [NG]";
    }

    LOG_INFO(3) << "Close CAN Channel.";
    if (Nullptr_t != can_channel_) {
      can_channel_->CloseChannel();
    }
  }

  return (true);
}

void TaskRecvImuData::ThreadReceivingImu() {
  LOG_INFO(3) << "Thread of receiving imu data ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFrame can_frame[20];
  // framework::Dormancy dormancy(10);

  while (running_flag_recv_imu_) {
#if 1
    frame_num = can_channel_->ReadWait(&(can_frame[0]), 50);
    // std::cout << "Received " << frame_num << " data from imu." << std::endl;

    if (frame_num > 0) {
      for (Int32_t i = 0; i < frame_num; ++i) {
        const can_dev::CanFrame &fm = can_frame[i];
#if 0
        printf("Imu: id[%08X] data[%02X %02X %02X %02X %02X %02X %02X %02X]\n",
               fm.id, fm.data[0], fm.data[1], fm.data[2], fm.data[3],
               fm.data[4], fm.data[5], fm.data[6], fm.data[7]);
#endif
        ParseCanFrame(fm);
#else
    imu_info_.Clear();
    imu_info_.msg_head.valid = true;
    imu_info_.msg_head.timestamp = common::GetClockNowMs();
    imu_info_.msg_head.UpdateSequenceNum();
#endif
        phoenix::framework::MessageRecvImuData message(&imu_info_);
        Notify(message);
        // dormancy.Sleep();
      }
    }
  }  // namespace wit_motion
  LOG_INFO(3) << "Thread of receiving imu data ... [Stopped]";
}  // namespace imu

void TaskRecvImuData::ParseCanFrame(const can_dev::CanFrame &frame) {
  const static Float32_t G = 9.8106F;
  int msg_id = frame.id;
  unsigned char msg_data[8] = {0};
  memcpy(msg_data, frame.data, frame.data_len);
  imu_info_.msg_head.valid = true;
  imu_info_.msg_head.timestamp = common::GetClockNowMs();
  switch (frame.data[1]) {
    case 0x50: {  //获取时间
                  // imu_.msg_head.timestamp =
      break;
      case 0x51:  //加速度
                  // ax=((AxH<<8)|AxL)/32768*16g(g为重力加速度,可取9.8m/s2)
        imu_info_.accel_x =
            (short)((frame.data[3] << 8) | frame.data[2]) / 32768.0 * 16 * G;
        imu_info_.accel_y =
            (short)((frame.data[5] << 8) | frame.data[4]) / 32768.0 * 16 * G;
        imu_info_.accel_z =
            (short)((frame.data[7] << 8) | frame.data[6]) / 32768.0 * 16 * G;
        break;
      case 0x52:  // wx=((wxH<<8)|wxL)/32768*2000(°/s)
        imu_info_.roll_rate = common::com_deg2rad(
            (short)((frame.data[3] << 8) | frame.data[2]) / 32768.0 * 2000.0);
        imu_info_.pitch_rate = common::com_deg2rad(
            (short)((frame.data[5] << 8) | frame.data[4]) / 32768.0 * 2000.0);
        imu_info_.yaw_rate = common::com_deg2rad(
            (short)((frame.data[7] << 8) | frame.data[6]) / 32768.0 * 2000.0);
        break;
      case 0x53:
        imu_info_.roll = common::com_deg2rad(
            (short)((frame.data[3] << 8) | frame.data[2]) / 32768.0 * 180.0);
        imu_info_.pitch = common::com_deg2rad(
            (short)((frame.data[5] << 8) | frame.data[4]) / 32768.0 * 180.0);
        imu_info_.yaw = common::com_deg2rad(
            (short)((frame.data[7] << 8) | frame.data[6]) / 32768.0 * 180.0);
        break;
      case 0x54:  //磁场信息，暂时不用
        break;
    }
  }
#if ENABLE_IMU_MSG_TRACE
  printf("accel: x=%f, y=%f, z=%f\n", imu_info_.accel_x, imu_info_.accel_y,
         imu_info_.accel_z);
  printf("roll_rate=%f, pitch_rate=%f, yaw_rate=%f\n", imu_info_.roll_rate,
         imu_info_.pitch_rate, imu_info_.yaw_rate);
  printf("roll=%f, pitch=%f, yaw=%f\n", imu_info_.roll, imu_info_.pitch,
         imu_info_.yaw);
#endif
}
}  // namespace wit_motion
}  // namespace imu
}  // namespace sensor
}  // namespace phoenix
