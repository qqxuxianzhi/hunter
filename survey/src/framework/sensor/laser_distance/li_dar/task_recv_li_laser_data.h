#ifndef TASK_RECV_LI_LASER_DATA_H
#define TASK_RECV_LI_LASER_DATA_H

#include <boost/atomic.hpp>
#include <boost/thread.hpp>

#include "ad_msg.h"
#include "common/task.h"
#include "serial_dev/serial_driver.h"
namespace phoenix {
namespace sensor {
namespace laser {


class TaskRecvLiLaserData : public framework::Task {
 public:
 
    enum {
        PKG_HEADER = 0x54,
        PKG_VER_LEN = 0x2C,
        POINT_PER_PACK = 12,
    };
    typedef struct __attribute__((packed)) {
        uint16_t distance;
        uint8_t intensity;
    } LidarPointStructDef;

    typedef struct __attribute__((packed)) {
        uint8_t header;
        uint8_t ver_len;
        uint16_t temperature;
        uint16_t start_angle;
        LidarPointStructDef point[POINT_PER_PACK];
        uint16_t end_angle;
        uint16_t timestamp;
        uint8_t crc8;
    } LiDARFrameTypeDef;
    

  TaskRecvLiLaserData(framework::Task* manager);
  ~TaskRecvLiLaserData();

  bool Start();
  bool Stop();

 private:
  void ThreadReceivingLaser();

  uint8_t CalCRC8(const uint8_t *data, uint16_t data_len);
  void SpinSerialData(const Uint8_t* buffer, const Int32_t length);
  void ParseMessageFrame();

 private:
  static const uint8_t CrcTable[256]; 
  LiDARFrameTypeDef pkg;

  serial_dev::SerialPortParam param;
  boost::atomic_bool running_flag_recv_laser_;
  boost::thread thread_recv_laser_;
  serial_dev::SerialDriver* serial_dev_;

  enum { MAX_DATA_BUFF_SIZE = 2048 };
  Uint8_t data_buff_[MAX_DATA_BUFF_SIZE];
};

}  // namespace laser
}  // namespace sensor
}  // namespace phoenix
#endif  // TASK_RECV_LI_LASER_DATA_H