#ifndef PHOENIX_AD_MSG_MSG_SURVEY_RESULT_H_
#define PHOENIX_AD_MSG_MSG_SURVEY_RESULT_H_

#include <string>

#include "msg_common.h"
#include "utils/com_utils.h"
#include "utils/macros.h"
namespace phoenix {
namespace ad_msg {

struct PointENU {
  Float64_t x;  // East from the origin, in meters.
  Float64_t y;  // North from the origin, in meters.
  Float64_t z;  // Up from the WGS-84 ellipsoid, in meters.
  void Clear() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }
};
struct PointLLH {
  Float64_t lon;     // Longitude in degrees, ranging from -180 to 180.
  Float64_t lat;     // Latitude in degrees, ranging from -90 to 90.
  Float64_t height;  // WGS-84 ellipsoid height in meters.

  void Clear() {
    lon = 0.0;
    lat = 0.0;
    height = 0.0;
  }
};

//测量请求
struct SurveyRequest {
  MsgHead msg_head;
  std::string index;  //桩号
  PointLLH request_point;
};

struct SurveyResult {
  MsgHead msg_head;
  std::string index;
  PointLLH result_point;
};

//激光测量结果
struct LaserResult {
  MsgHead msg_head;
  Float64_t height;  //激光测距 高度测量
  void Clear() {
    msg_head.valid = false;
    height = 0.0;
  }
};

// Rtk测量结果
struct RtkResult {
  MsgHead msg_head;
  PointENU utm_result;
  PointLLH gnss_result;
  void Clear() {
    msg_head.valid = false;
    utm_result.Clear();
    gnss_result.Clear();
  }
};
//激光测量结果(加料机构测距)
struct LaserLiResult{
 // Polar coordinate representation
  float angle;         // Angle ranges from 0 to 359 degrees
  uint16_t distance;   // Distance is measured in millimeters(mm)
  uint8_t intensity;  // Intensity is 0 to 255 强度
  // Cartesian coordinate representation
  
  double x;          // LD TOFLiDAR no support x an y param
  double y;
  LaserLiResult(float angle, uint16_t distance, uint8_t intensity, double x = 0,
            double y = 0) {
    this->angle = angle;
    this->distance = distance;
    this->intensity = intensity;
    this->x = x;
    this->y = y;
  }
  LaserLiResult() {}
  void clear(){
    angle = 0.0;
    distance =0;
    intensity =0;
    x =0;
    y =0;
  }
  friend std::ostream &operator<<(std::ostream &os, const LaserLiResult &data) {
    os << "angle: " << data.angle << " distance: " << data.distance << "  intensity: " << (int)data.intensity;
    return os;
  }
};

}  // namespace ad_msg
}  // namespace phoenix
#endif