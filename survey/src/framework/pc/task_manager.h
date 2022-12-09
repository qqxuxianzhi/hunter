/******************************************************************************
 ** 任务管理模块
 ******************************************************************************
 *
 *  管理所有的任务(启动、停止、状态监测等)
 *
 *  @file       task_manager.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_TASK_MANAGER_H_
#define PHOENIX_FRAMEWORK_TASK_MANAGER_H_

#include <boost/atomic.hpp>
#include <boost/thread.hpp>
#include <memory>
#include <string>

#include "common/task.h"
#include "os/mutex.h"
#include "util.h"
#include "utils/com_utils.h"
#include "work/work_monitor.h"

namespace phoenix {

namespace sensor {
namespace imu {
#if (DEV_IMU_TYPE == DEV_IMU_TYPE_MPSK)
namespace mpsk {
class TaskRecvGnssDataMpsk;
}
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_BDSTAR)
namespace bdstar {
class TaskRecvGnssDataBdstar;
}
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_INTERTIAL_LAB)
namespace intertial_lab {
class TaskRecvGnssIntertialLab;
}
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_F9K)
namespace f9k {
class TaskRecvNmeaData;
}
#else
// 未定义IMU设备
#endif
//姿态传感器
namespace wit_motion {
class TaskRecvImuData;
}
}  // namespace imu

#if (DEV_POWDER_LASER_TYPE == DEV_POWDER_LASER_TYPE_LI)
namespace laser {
class TaskRecvLiLaserData;
}
#endif

#if (DEV_LASER_TYPE == DEV_LASER_TYPE_RANGRING)
namespace laser {
class TaskRecvLaserData;
}
#elif (DEV_LASER_TYPE == DEV_LASER_TYPE_JRT)
namespace laser {
class TaskRecvJRTLaserData;
}
#else

#endif

#if (DEV_IMU_TYPE == DEV_IMU_TYPE_ZHD_RTK)
namespace rtk {
class TaskRecvRtkData;
}
#endif
}  // namespace sensor

namespace framework {
class RosNode;
class LcmNode;
class UdpNode;
class MsgReceiver;
class MsgSender;
class MsgServer;

class TaskManager : public Task {
 public:
  TaskManager(int argc, char** argv, const std::string& work_space);
  ~TaskManager();

  enum {
    START_OK = 0,
    START_ERR_FAILED_TO_START_ROS,
    START_ERR_FAILED_TO_START_LCM,
    START_ERR_FAILED_TO_START_UDP,
    START_ERR_FAILED_TO_START_MSG_RECV,
    START_ERR_FAILED_TO_START_MSG_SEND,
    START_ERR_FAILED_TO_START_GNSS,
    START_ERR_FAILED_TO_START_MSG_SERVER,
    START_ERR
  };
  Int32_t Start();
  bool Stop();

 private:
  void ThreadCheckTasksStatus();
  bool HandleMessage(const Message& msg, Task* sender) override;
  bool ReadSvg();
  bool ReadLocalConfigFromFile();

private:
  typedef boost::unique_lock<boost::mutex> Lock;

#if (ENABLE_ROS_NODE)
  std::unique_ptr<RosNode> ros_node_;
#endif
#if (ENABLE_LCM_NODE)
  std::unique_ptr<LcmNode> lcm_node_;
#endif
#if (ENABLE_UDP_NODE)
  std::unique_ptr<UdpNode> udp_node_;
#endif
  std::unique_ptr<MsgReceiver> msg_receiver_;
  std::unique_ptr<MsgSender> msg_sender_;
  std::unique_ptr<MsgServer> msg_server_;
#if (DEV_IMU_TYPE == DEV_IMU_TYPE_MPSK)
  std::unique_ptr<sensor::imu::mpsk::TaskRecvGnssDataMpsk> task_recv_gnss_data_;
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_BDSTAR)
  std::unique_ptr<sensor::imu::bdstar::TaskRecvGnssDataBdstar>
      task_recv_gnss_data_;
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_INTERTIAL_LAB)
  std::unique_ptr<sensor::imu::intertial_lab::TaskRecvGnssIntertialLab>
      task_recv_gnss_data_;
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_F9K)
  std::unique_ptr<sensor::imu::f9k::TaskRecvNmeaData> task_recv_gnss_data_;
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_ZHD_RTK)
  std::unique_ptr<sensor::rtk::TaskRecvRtkData> task_recv_gnss_data_;
#else
  // 未定义IMU设备
#endif

  //姿态传感器
  // std::unique_ptr<sensor::imu::wit_motion::TaskRecvImuData>
  // task_recv_imu_data_;
  //激光测距传感器
#if (DEV_LASER_TYPE == DEV_LASER_TYPE_RANGRING)
  std::unique_ptr<sensor::laser::TaskRecvLaserData> task_recv_laser_data_;
#elif (DEV_LASER_TYPE == DEV_LASER_TYPE_JRT)
  std::unique_ptr<sensor::laser::TaskRecvJRTLaserData> task_recv_laser_data_;
#else
#endif

#if (DEV_POWDER_LASER_TYPE == DEV_POWDER_LASER_TYPE_LI)
 std::unique_ptr<sensor::laser::TaskRecvLiLaserData> task_powder_li_laser_data_;
 #else
#endif

  boost::atomic_bool thread_running_flag_check_tasks_status_;
  boost::thread thread_check_tasks_status_;

 private:
  WorkMonitor work_monitor_;

  std::string work_space_;

  // log
  char str_buff_status_[1024 * 2];
};

}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_TASK_MANAGER_H_
