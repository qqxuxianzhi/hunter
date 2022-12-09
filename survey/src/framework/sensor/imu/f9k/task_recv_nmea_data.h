#ifndef TASK_RECV_NMEA_DATA_H
#define TASK_RECV_NMEA_DATA_H

#include <boost/atomic.hpp>
#include <boost/thread.hpp>
#include <memory>
#include <queue>

#include "ad_msg.h"
#include "common/task.h"
#include "serial_dev/serial_driver.h"

namespace phoenix {
namespace sensor {
namespace imu {
namespace f9k {

class TaskRecvNmeaData : public framework::Task {
 public:
  TaskRecvNmeaData(framework::Task* manager);
  ~TaskRecvNmeaData();

  bool Start();
  bool Stop();

 private:
  void ThreadReceivingNmea();

  void SpinSerialData(const Uint8_t* buffer, Int32_t length);

  // void ParseMessageFrame(const Uint8_t* buffer, Int32_t length);

  void ParseGGA(std::string str_data);
  void ParseRMC(std::string str_data);

 private:
  typedef boost::unique_lock<boost::mutex> Lock;

  boost::atomic_bool running_flag_recv_nmea_;
  boost::thread thread_recv_nmea_;

  serial_dev::SerialDriver* serial_dev_;

  enum { MAX_DATA_BUFF_SIZE = 2048 };
  Uint8_t data_buff_[MAX_DATA_BUFF_SIZE];
  Int32_t data_length_;
  std::string str_data_buff_;
  phoenix::ad_msg::Gnss gnss_info_;
};
}  // namespace f9k
}  // namespace imu
}  // namespace sensor
}  // namespace phoenix
#endif  // TASK_RECV_NMEA_DATA_H