#include "sensor/imu/f9k/task_recv_nmea_data.h"

#include <boost/algorithm/string.hpp>

#include "communication/shared_data.h"
#include "utils/com_utils.h"
#include "utils/gps_tools.h"
#include "utils/log.h"
#if (ENABLE_SERIAL_DEV_POSIX)
#include "serial_dev/posix/serial_driver_posix.h"
#endif
#if (ENABLE_SERIAL_DEV_MDC)
#include "serial_dev/mdc/serial_driver_mdc.h"
#endif

namespace phoenix {
namespace sensor {
namespace imu {
namespace f9k {
// std::ofstream outGPSSStream("GPSPosition.txt", std::ios::app);
// std::ofstream outGPSAngleStream("GPSAngle.txt", std::ios::app);
// std::ofstream outGPSTime("GPSTime.txt",std::ios::app);

TaskRecvNmeaData::TaskRecvNmeaData(framework::Task *manager)
    : framework::Task(framework::TASK_ID_RECV_GNSS_DATA, "Recv Nmea Data",
                      manager) {
  running_flag_recv_nmea_ = false;

  // serial_dev_ = Nullptr_t;
  serial_dev_ = new serial_dev::SerialDriverPosix();
  data_length_ = 0;
  common::com_memset(data_buff_, 0, sizeof(data_buff_));
}

TaskRecvNmeaData::~TaskRecvNmeaData() {
  Stop();
  if (Nullptr_t != serial_dev_) {
    delete serial_dev_;
  }
}

bool TaskRecvNmeaData::Start() {
  if (Nullptr_t == serial_dev_) {
    LOG_ERR << "Invalid serial device.";
    return false;
  }
  if (running_flag_recv_nmea_) {
    return (true);
  }
  serial_dev::SerialPortParam param;
  com_snprintf(param.port_name, 64, "/dev/ttyUSB1");
  param.data_bits = serial_dev::SerialPortParam::DATA_BITS_8;
  param.parity = serial_dev::SerialPortParam::PARITY_NO;
  param.stop_bits = serial_dev::SerialPortParam::STOP_BITS_ONE;
  param.baud_rate = serial_dev::SerialPortParam::BAUD_RATE_38400;

  if (!serial_dev_->OpenPort(param)) {
    LOG_ERR << "Failed to open serial device.";
    // return false;
  }

  data_length_ = 0;
  common::com_memset(data_buff_, 0, sizeof(data_buff_));

  LOG_INFO(3) << "Create thread of receiving nmea data...";
  running_flag_recv_nmea_ = true;
  thread_recv_nmea_ =
      boost::thread(boost::bind(&TaskRecvNmeaData::ThreadReceivingNmea, this));

  LOG_INFO(3) << "Create thread of receiving nmea data... [OK]";

  return (true);
}

bool TaskRecvNmeaData::Stop() {
  if (running_flag_recv_nmea_) {
    running_flag_recv_nmea_ = false;

    LOG_INFO(3) << "Stop thread of receiving nmea data ...";
    bool ret = thread_recv_nmea_.timed_join(boost::posix_time::seconds(1));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread of receiving nmea data to stop.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop thread of receiving nmea data ... [OK]";
    } else {
      LOG_INFO(3) << "Stop thread of receiving nmea data ... [NG]";
    }

    LOG_INFO(3) << "Close serial port.";
    if (Nullptr_t != serial_dev_) {
      serial_dev_->ClosePort();
    }
  }
  return (true);
}

void TaskRecvNmeaData::ThreadReceivingNmea() {
  LOG_INFO(3) << "Thread of receiving nmea data ... [Started]";

  Uint8_t data_buff[512];
  // framework::Dormancy dormancy(1000);

  while (running_flag_recv_nmea_) {
#if 1
    Int32_t bytes = serial_dev_->ReadWait(data_buff, 510);
    if (bytes > 0) {
      SpinSerialData(data_buff, bytes);
    }
#else
    static Float64_t offset = 0.1F;
    gnss_info_.Clear();
    gnss_info_.msg_head.valid = true;
    gnss_info_.msg_head.UpdateSequenceNum();
    gnss_info_.msg_head.timestamp = common::GetClockNowMs();
    gnss_info_.latitude = 30.409097494F;
    gnss_info_.longitude = 114.442950572F;
    gnss_info_.altitude = 19.7507F;  //使用水准面
    gnss_info_.x_utm = 254356.2711F - 1;
    gnss_info_.y_utm = 3366893.7695F + 1;
    gnss_info_.gnss_status = ad_msg::Gnss::STATUS_GOOD;
    gnss_info_.utm_status = ad_msg::Gnss::STATUS_GOOD;
    gnss_info_.heading_gnss = 2.217798F;
    gnss_info_.heading_utm = 2.217798F;
    offset += 0.2;
#endif
    phoenix::framework::MessageRecvGnssData message(&gnss_info_);
    Notify(message);

    // dormancy.Sleep();
  }
  LOG_INFO(3) << "Thread of receiving nmea data ... [Stopped]";
}

void TaskRecvNmeaData::SpinSerialData(const Uint8_t *buffer, Int32_t length) {
  std::string databuf((char *)buffer, length);
  str_data_buff_ += databuf;
  int start_pos = str_data_buff_.find_first_of("$");
  if (start_pos == str_data_buff_.npos) {
    str_data_buff_.clear();
  } else {
    int end_pos = str_data_buff_.find_first_of("*", start_pos);
    if (end_pos != str_data_buff_.npos &&
        str_data_buff_.size() > (end_pos + 3)) {
      std::string str_line =
          str_data_buff_.substr(start_pos, end_pos - start_pos + 3);
      if (str_line.at(5) == 'A') {
        // std::cout << "GGA:" << str_line << std::endl;
        ParseGGA(str_line);
      }
      if (str_line.at(5) == ('C')) {
        ParseRMC(str_line);
        // std::cout << "RMC:" << str_line << std::endl;
      }
      str_data_buff_ = str_data_buff_.substr(end_pos + 3);
    }
  }
}

void TaskRecvNmeaData::ParseGGA(std::string str_data) {
  std::vector<std::string> gga_data;
  boost::split(gga_data, str_data, boost::is_any_of(","),
               boost::token_compress_off);
  if (gga_data.size() == 15 && !gga_data[2].empty() && !gga_data[4].empty()) {
    gnss_info_.msg_head.valid = true;
    gnss_info_.msg_head.UpdateSequenceNum();
    gnss_info_.msg_head.timestamp = common::GetClockNowMs();
    int dd = atoi(gga_data[2].substr(0, 2).c_str());
    Float64_t mm = atof(gga_data[2].substr(2).c_str()) / 60.0;
    Float64_t lat = dd + mm;

    int ddd = atoi(gga_data[4].substr(0, 3).c_str());
    Float64_t mmm = atof(gga_data[4].substr(3).c_str()) / 60.0;
    Float64_t lon = ddd + mmm;
    gnss_info_.latitude = lat;
    gnss_info_.longitude = lon;
    gnss_info_.altitude = atof(gga_data[11].c_str());  //使用水准面
    Float64_t utm_x, utm_y;
    common::LLtoUTM(lat, lon, &utm_y, &utm_x);
    gnss_info_.x_utm = utm_x;
    gnss_info_.y_utm = utm_y;

    int gnss_status = atoi(gga_data[6].c_str());
    if (gnss_status == 4 || gnss_status == 5 || gnss_status == 6) {
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_GOOD;
    } else if (gnss_status == 2) {
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_CONVERGING;
    } else {
      gnss_info_.gnss_status = ad_msg::Gnss::STATUS_BAD;
    }
    gnss_info_.utm_status = gnss_info_.gnss_status;
#if 0
    std::cout << std::setprecision(15)
              << "time:" << gnss_info_.msg_head.timestamp << std::endl;
    std::cout << std::setprecision(15) << "lat:" << gnss_info_.latitude
              << std::endl;
    std::cout << std::setprecision(15) << "lon:" << gnss_info_.longitude
              << std::endl;
    std::cout << std::setprecision(15) << "x_utm:" << gnss_info_.x_utm
              << std::endl;
    std::cout << std::setprecision(15) << "y_utm:" << gnss_info_.y_utm
              << std::endl;
#endif
  }
}

void TaskRecvNmeaData::ParseRMC(std::string str_data) {
  std::vector<std::string> rmc_data;
  boost::split(rmc_data, str_data, boost::is_any_of(","),
               boost::token_compress_off);
  if (rmc_data.size() == 14 && !rmc_data[10].empty() && !rmc_data[11].empty()) {
//解析时间
#if 0
    struct tm t;
    t.tm_year = atoi(rmc_data[9].substr(4, 2).c_str()) + 2000 - 1900;
    t.tm_mon = atoi(rmc_data[9].substr(2, 2).c_str()) - 1;
    t.tm_mday = atoi(rmc_data[9].substr(0, 2).c_str());
    t.tm_hour = atoi(rmc_data[1].substr(0, 2).c_str()) + 8;
    t.tm_min = atoi(rmc_data[1].substr(2, 2).c_str());
    t.tm_sec = atoi(rmc_data[1].substr(4, 2).c_str());
    // std::cout << t.tm_year << "-" << t.tm_mon << "-" << t.tm_mday << " "
    //          << t.tm_hour << ":" << t.tm_min << ":" << t.tm_sec << std::endl;
    int timestamp = mktime(&t);
    std::cout << timestamp << std::endl;
#endif
    //解析航向,GPRMC的速度单位是节
    // if (atof(rmc_data[7].c_str()) > 0.1) {
    // 1节=0.54m/s 速度>0.1m/s时，采用航向角
    gnss_info_.heading_gnss =
        common::com_deg2rad(common::ConvGpsHeading(atof(rmc_data[8].c_str())));
    //} else {
    //静态时采用磁偏角
    // if (rmc_data[11] == "E") {
    //向东偏，顺时针为正
    // gnss_info_.heading_gnss = common::com_deg2rad(
    //    common::ConvGpsHeading(atof(rmc_data[10].c_str())));
    //} else {
    //向西偏，逆时针为负
    // gnss_info_.heading_gnss = common::com_deg2rad(
    //   common::ConvGpsHeading(-atof(rmc_data[10].c_str())));
    // }
    //}
    gnss_info_.heading_utm = gnss_info_.heading_gnss;
  }
}
}  // namespace f9k
}  // namespace imu
}  // namespace sensor
}  // namespace phoenix