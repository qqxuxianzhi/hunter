#include "task_recv_rtk_data.h"

#include <boost/algorithm/string.hpp>
#include <chrono>

#include "pc/util.h"
#include "serial_dev/posix/serial_driver_posix.h"
#include "utils/gps_tools.h"
namespace phoenix {
namespace sensor {
namespace rtk {
TaskRecvRtkData::TaskRecvRtkData(framework::Task *manager)
    : framework::Task(framework::TASK_ID_RECV_RTK_DATA, "Recv Rtk Data",
                      manager) {
  running_flag_recv_rtk_ = false;
  common::com_memset(data_buff_, 0, sizeof(data_buff_));
  tcp_client_ = Nullptr_t;
  tcp_client_ = new framework::TcpClient();
}

TaskRecvRtkData::~TaskRecvRtkData() {
  Stop();
  if (Nullptr_t != tcp_client_) {
    delete tcp_client_;
  }
}

bool TaskRecvRtkData::Start() {
  if (running_flag_recv_rtk_) {
    return (true);
  }

  common::com_memset(data_buff_, 0, sizeof(data_buff_));

  if (Nullptr_t != tcp_client_) {
    framework::TcpClient::TcpClientParam param;
    param.server_port = 4545;
    param.connect_timeout_ms = 3000;
    param.enable = true;
    strncpy(param.server_addr, "192.168.20.1", sizeof(param.server_addr));
    if ((!tcp_client_->Start(param)) || (!tcp_client_->StartReceiving())) {
      tcp_client_->Stop();
      LOG_ERR << "Falied to open tcp client and close connect.";
      return (false);
    }
    Uint8_t buf[20] = "gpgga 1\r\n";
    tcp_client_->Send(&buf[0], 9);
    tcp_client_->SetRecvMsgCallback(
        std::bind(&TaskRecvRtkData::SpinSerialData, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));
  } else {
    LOG_ERR << "Failed to tcp client is null ptr.";
    return (false);
  }
  running_flag_recv_rtk_ = true;

  return (true);
}

bool TaskRecvRtkData::Stop() {
  if (running_flag_recv_rtk_) {
    running_flag_recv_rtk_ = false;

    if (Nullptr_t != tcp_client_) {
      LOG_INFO(3) << "Close Tcp Client.";
      tcp_client_->Stop();
    }
  }
  return (true);
}

void TaskRecvRtkData::SpinSerialData(const Uint8_t *buffer,
                                     const Int32_t length,
                                     const Int64_t receive_time) {
  std::string databuf((char *)buffer, length);
  if (databuf.size() > 0) {
    middle_layer_pb::RtkDataSource rtk_data_source;
    rtk_data_source.set_data(databuf);
    phoenix::framework::MessageRtkDataSource rtk_msg(&rtk_data_source);
    Notify(rtk_msg);
  }

  str_data_buff_ += databuf;
  int start_pos = str_data_buff_.find("$GPGGA");
  if (start_pos == str_data_buff_.npos) {
    str_data_buff_.clear();
  } else {
    int end_pos = str_data_buff_.find_first_of("*", start_pos);
    if (end_pos != str_data_buff_.npos &&
        str_data_buff_.size() > (end_pos + 3)) {
      std::string str_line =
          str_data_buff_.substr(start_pos, end_pos - start_pos + 3);
      if (str_line.find("GGA") != str_line.npos) {
        std::cout << "ZHD:" << str_line << std::endl;
        ParseGGA(str_line);
      }
      if (rtk_result_.msg_head.valid) {
        phoenix::framework::MessageRtkData message(&rtk_result_);
        Notify(message);
      }
      str_data_buff_ = str_data_buff_.substr(end_pos + 3);
    }
  }
}

void TaskRecvRtkData::ParseGGA(std::string str_data) {
  rtk_result_.Clear();
  std::vector<std::string> gga_data;
  boost::split(gga_data, str_data, boost::is_any_of(","),
               boost::token_compress_off);
  if (gga_data.size() == 15 && !gga_data[2].empty() && !gga_data[4].empty()) {
    int gnss_status = atoi(gga_data[6].c_str());  //获取差分状态
    if (gnss_status == 4) {                       // RTK获取到固定解
      rtk_result_.msg_head.valid = true;
      rtk_result_.msg_head.UpdateSequenceNum();
      rtk_result_.msg_head.timestamp = common::GetClockNowMs();

      int dd = atoi(gga_data[2].substr(0, 2).c_str());
      Float64_t mm = atof(gga_data[2].substr(2).c_str()) / 60.0;
      Float64_t lat = dd + mm;

      int ddd = atoi(gga_data[4].substr(0, 3).c_str());
      Float64_t mmm = atof(gga_data[4].substr(3).c_str()) / 60.0;
      Float64_t lon = ddd + mmm;

      rtk_result_.gnss_result.lat = lat;
      rtk_result_.gnss_result.lon = lon;
      rtk_result_.gnss_result.height =
          atof(gga_data[11].c_str());  //地球椭球面相对大地水准面的高度
      Float64_t utm_x, utm_y;
      common::LLtoUTM(lat, lon, &utm_y, &utm_x);
      rtk_result_.utm_result.x = utm_x;
      rtk_result_.utm_result.y = utm_y;
      rtk_result_.utm_result.z = atof(gga_data[9].c_str());  //海拔高度
    }
  }
}
}  // namespace rtk
}  // namespace sensor
}  // namespace phoenix
