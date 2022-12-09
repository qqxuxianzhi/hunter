#ifndef TASK_RECV_LASER_DATA_H
#define TASK_RECV_LASER_DATA_H

#include <boost/atomic.hpp>
#include <boost/thread.hpp>

#include "ad_msg.h"
#include "common/task.h"
#include "serial_dev/serial_driver.h"
namespace phoenix {
namespace sensor {
namespace laser {
class TaskRecvLaserData : public framework::Task {
 public:
  TaskRecvLaserData(framework::Task* manager);
  ~TaskRecvLaserData();

  bool Start();
  bool Stop();

 private:
  void ThreadReceivingLaser();

  Float64_t SpinSerialData(const Uint8_t* buffer, const Int32_t length);

  void OpenLaser();
  void CloseLaser();
  void StartLaserRanging();
  void SetFreq();  // FA 04 0A 0A EE
  // void ParseMessageFrame(const Uint8_t* buffer, Int32_t length);

  // void ParseGGA(std::string str_data);
  // void ParseRMC(std::string str_data);

 private:
  boost::atomic_bool running_flag_recv_laser_;
  boost::thread thread_recv_laser_;
  serial_dev::SerialDriver* serial_dev_;

  enum { MAX_DATA_BUFF_SIZE = 2048 };
  Uint8_t data_buff_[MAX_DATA_BUFF_SIZE];
};
}  // namespace laser
}  // namespace sensor
}  // namespace phoenix
#endif  // TASK_RECV_LASER_DATA_H
