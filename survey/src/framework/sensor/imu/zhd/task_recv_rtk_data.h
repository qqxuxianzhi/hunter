#ifndef TASK_RECV_RTK_DATA_H
#define TASK_RECV_RTK_DATA_H

#include <boost/atomic.hpp>
#include <boost/thread.hpp>

#include "ad_msg.h"
#include "common/task.h"
#include "communication/tcp_client.h"
#include "serial_dev/serial_driver.h"
namespace phoenix {
namespace sensor {
namespace rtk {

class TaskRecvRtkData : public framework::Task {
 public:
  TaskRecvRtkData(framework::Task* manager);
  ~TaskRecvRtkData();

  bool Start();
  bool Stop();

 private:
  void SpinSerialData(const Uint8_t* buffer, const Int32_t length,
                      const Int64_t receive_time);

  void ParseGGA(std::string str_data);

 private:
  boost::atomic_bool running_flag_recv_rtk_;
  framework::TcpClient* tcp_client_;

  enum { MAX_DATA_BUFF_SIZE = 2048 };
  Uint8_t data_buff_[MAX_DATA_BUFF_SIZE];
  std::string str_data_buff_;
  phoenix::ad_msg::RtkResult rtk_result_;
};
}  // namespace rtk
}  // namespace sensor
}  // namespace phoenix
#endif  // TASK_RECV_RTK_DATA_H
