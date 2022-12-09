#ifndef PHOENIX_SENSOR_TASK_RECV_IMU_DATA_H_
#define PHOENIX_SENSOR_TASK_RECV_IMU_DATA_H_

#include <boost/atomic.hpp>
#include <boost/thread.hpp>
#include <memory>

#include "ad_msg.h"
#include "can_dev/can_driver.h"
#include "common/task.h"
#include "geometry/geometry_utils.h"
#include "math/math_utils.h"
#include "math/matrix.h"

namespace phoenix {
namespace sensor {
namespace imu {
namespace wit_motion {
class TaskRecvImuData : public framework::Task {
 public:
  explicit TaskRecvImuData(framework::Task* manager);
  ~TaskRecvImuData();

  bool Start();
  bool Stop();

 private:
  void ThreadReceivingImu();

  void ParseCanFrame(const can_dev::CanFrame& frame);

 private:
  typedef boost::unique_lock<boost::mutex> Lock;

  can_dev::CanDriver* can_channel_;

  boost::atomic_bool running_flag_recv_imu_;
  boost::thread thread_recv_imu_;

  phoenix::ad_msg::Imu imu_info_;
};
}  // namespace wit_motion
}  // namespace imu
}  // namespace sensor
}  // namespace phoenix

#endif  // PHOENIX_SENSOR_TASK_RECV_IMU_DATA_H_
