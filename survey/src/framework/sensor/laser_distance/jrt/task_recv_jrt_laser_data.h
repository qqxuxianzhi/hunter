#ifndef TASK_RECV_JRT_LASER_DATA_H
#define TASK_RECV_JRT_LASER_DATA_H

#include <boost/atomic.hpp>
#include <boost/thread.hpp>

#include "ad_msg.h"
#include "common/task.h"
#include "serial_dev/serial_driver.h"
namespace phoenix {
namespace sensor {
namespace laser {
class TaskRecvJRTLaserData : public framework::Task {
 public:
  TaskRecvJRTLaserData(framework::Task* manager);
  ~TaskRecvJRTLaserData();

  bool Start();
  bool Stop();
  Uint8_t GetLaserStatus();
  void OpenLaser();
  void CloseLaser();
  void CntinusAuto();
  void CntinusExit();

  enum {
    LASER_ON,           //激光开启
    LASER_OPENING,      //已发送开启激光器命令
    LASER_OFF,          //激光关闭
    LASER_CLOSING,      //已发送激光器关闭命令
    LASER_AUTO_CNT,     //开启连续测量
    LASER_AUTO_CNTING,  //已发送开启连续测量命令
    LASER_EXIT_CNT      //关闭连续测量
  };
  Uint8_t is_laser_status_;

 private:
  void ThreadReceivingLaser();

  void SpinSerialData(const Uint8_t* buffer, const Int32_t length);
  void ParseMessageFrame(const Uint8_t* buffer, Int32_t length);

  void OneShotSlow();  //单次慢速测量
 private:
  boost::atomic_bool running_flag_recv_laser_;
  boost::thread thread_recv_laser_;
  serial_dev::SerialDriver* serial_dev_;

  enum { MAX_DATA_BUFF_SIZE = 2048 };
  Uint8_t data_buff_[MAX_DATA_BUFF_SIZE];
  Int32_t data_length_;
};
}  // namespace laser
}  // namespace sensor
}  // namespace phoenix
#endif  // TASK_RECV_JRT_LASER_DATA_H
