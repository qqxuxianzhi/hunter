//
#include "communication/parse_proto_msg.h"

#include "math/math_utils.h"
#include "utils/gps_tools.h"
#include "utils/log.h"

namespace phoenix {
namespace framework {

bool ParseProtoMsg::EncodeGnssMessage(const ad_msg::Gnss& msg,
                                      msg::localization::Gnss* const data_out) {
  data_out->set_latitude(msg.latitude);
  data_out->set_longitude(msg.longitude);
  data_out->set_altitude(msg.altitude);
  data_out->set_heading_gnss(msg.heading_gnss);
  data_out->set_v_e(msg.v_e);
  data_out->set_v_n(msg.v_n);
  data_out->set_v_u(msg.v_u);
  data_out->set_pitch(msg.pitch);
  data_out->set_roll(msg.roll);
  data_out->set_x_utm(msg.x_utm);
  data_out->set_y_utm(msg.y_utm);
  data_out->set_z_utm(msg.z_utm);
  data_out->set_heading_utm(msg.heading_utm);
  data_out->set_v_x_utm(msg.v_x_utm);
  data_out->set_v_y_utm(msg.v_y_utm);
  data_out->set_v_z_utm(msg.v_z_utm);
  switch (msg.gnss_status) {
    case (ad_msg::Gnss::STATUS_BAD):
      data_out->set_gnss_status(msg::localization::Gnss::STATUS_BAD);
      break;
    case (ad_msg::Gnss::STATUS_CONVERGING):
      data_out->set_gnss_status(msg::localization::Gnss::STATUS_CONVERGING);
      break;
    case (msg::localization::Gnss::STATUS_GOOD):
      data_out->set_gnss_status(msg::localization::Gnss::STATUS_GOOD);
      break;
    default:
      data_out->set_gnss_status(msg::localization::Gnss::STATUS_INVALID);
      break;
  }
  switch (msg.utm_status) {
    case (ad_msg::Gnss::STATUS_BAD):
      data_out->set_utm_status(msg::localization::Gnss::STATUS_BAD);
      break;
    case (ad_msg::Gnss::STATUS_CONVERGING):
      data_out->set_utm_status(msg::localization::Gnss::STATUS_CONVERGING);
      break;
    case (msg::localization::Gnss::STATUS_GOOD):
      data_out->set_utm_status(msg::localization::Gnss::STATUS_GOOD);
      break;
    default:
      data_out->set_utm_status(msg::localization::Gnss::STATUS_INVALID);
      break;
  }
  switch (msg.odom_status) {
    case (ad_msg::Gnss::STATUS_BAD):
      data_out->set_odom_status(msg::localization::Gnss::STATUS_BAD);
      break;
    case (ad_msg::Gnss::STATUS_CONVERGING):
      data_out->set_odom_status(msg::localization::Gnss::STATUS_CONVERGING);
      break;
    case (msg::localization::Gnss::STATUS_GOOD):
      data_out->set_odom_status(msg::localization::Gnss::STATUS_GOOD);
      break;
    default:
      data_out->set_odom_status(msg::localization::Gnss::STATUS_INVALID);
      break;
  }

  return true;
}

bool ParseProtoMsg::DecodeGnssMessage(const Char_t* msg, Int32_t msg_len,
                                      ad_msg::Gnss* data_out) {
  // parse
  msg::localization::Gnss gnss;
  if (!gnss.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse gnss from array.";
    return false;
  }

  data_out->latitude = gnss.latitude();
  data_out->longitude = gnss.longitude();
  data_out->altitude = gnss.altitude();
  data_out->heading_gnss =
      common::com_deg2rad(common::ConvGpsHeading(gnss.heading_gnss()));
  data_out->x_utm = gnss.x_utm();
  data_out->y_utm = gnss.y_utm();
  data_out->z_utm = gnss.z_utm();
  /// TODO: UTM定位的航向角超出了-pi到+pi的范围，后期需要修改
  data_out->heading_utm = common::NormalizeAngle(gnss.heading_utm());
  data_out->x_odom = gnss.x_odom();
  data_out->y_odom = gnss.y_odom();
  data_out->z_odom = gnss.z_odom();
  data_out->heading_odom = common::NormalizeAngle(gnss.heading_odom());
  data_out->pitch = gnss.pitch();
  data_out->roll = gnss.roll();
  data_out->v_e = gnss.v_e();
  data_out->v_n = gnss.v_n();
  data_out->v_u = gnss.v_u();
  data_out->v_x_utm = gnss.v_x_utm();
  data_out->v_y_utm = gnss.v_y_utm();
  data_out->v_z_utm = gnss.v_z_utm();
  data_out->v_x_odom = gnss.v_x_odom();
  data_out->v_y_odom = gnss.v_y_odom();
  data_out->v_z_odom = gnss.v_z_odom();

  switch (gnss.gnss_status()) {
    case msg::localization::Gnss::STATUS_BAD:
      data_out->gnss_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case msg::localization::Gnss::STATUS_CONVERGING:
      data_out->gnss_status = ad_msg::Gnss::STATUS_CONVERGING;
      break;
    case msg::localization::Gnss::STATUS_GOOD:
      data_out->gnss_status = ad_msg::Gnss::STATUS_GOOD;
      break;
    default:
      data_out->gnss_status = ad_msg::Gnss::STATUS_INVALID;
      break;
  }

  switch (gnss.utm_status()) {
    case msg::localization::Gnss::STATUS_BAD:
      data_out->utm_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case msg::localization::Gnss::STATUS_CONVERGING:
      data_out->utm_status = ad_msg::Gnss::STATUS_CONVERGING;
      break;
    case msg::localization::Gnss::STATUS_GOOD:
      data_out->utm_status = ad_msg::Gnss::STATUS_GOOD;
      break;
    default:
      data_out->utm_status = ad_msg::Gnss::STATUS_INVALID;
      break;
  }

  switch (gnss.odom_status()) {
    case msg::localization::Gnss::STATUS_BAD:
      data_out->odom_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case msg::localization::Gnss::STATUS_CONVERGING:
      data_out->odom_status = ad_msg::Gnss::STATUS_CONVERGING;
      break;
    case msg::localization::Gnss::STATUS_GOOD:
      data_out->odom_status = ad_msg::Gnss::STATUS_GOOD;
      break;
    default:
      data_out->odom_status = ad_msg::Gnss::STATUS_INVALID;
      break;
  }

  if (com_isnan(data_out->latitude) || com_isinf(data_out->latitude) ||
      com_isnan(data_out->longitude) || com_isinf(data_out->longitude) ||
      com_isnan(data_out->heading_gnss) || com_isinf(data_out->heading_gnss)) {
    data_out->gnss_status = ad_msg::Gnss::STATUS_INVALID;

    data_out->latitude = 0;
    data_out->longitude = 0;
    data_out->heading_gnss = 0;
  }

  if (com_isnan(data_out->x_utm) || com_isinf(data_out->x_utm) ||
      com_isnan(data_out->y_utm) || com_isinf(data_out->y_utm) ||
      com_isnan(data_out->heading_utm) || com_isinf(data_out->heading_utm)) {
    data_out->utm_status = ad_msg::Gnss::STATUS_INVALID;

    data_out->x_utm = 0;
    data_out->y_utm = 0;
    data_out->heading_utm = 0;
  }

  if (com_isnan(data_out->x_odom) || com_isinf(data_out->x_odom) ||
      com_isnan(data_out->y_odom) || com_isinf(data_out->y_odom) ||
      com_isnan(data_out->heading_odom) || com_isinf(data_out->heading_odom)) {
    data_out->odom_status = ad_msg::Gnss::STATUS_INVALID;

    data_out->x_odom = 0;
    data_out->y_odom = 0;
    data_out->heading_odom = 0;
  }

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}

bool ParseProtoMsg::EncodeImuMessage(const ad_msg::Imu& msg,
                                     msg::localization::Imu* const data_out) {
  data_out->set_accel_x(msg.accel_x);
  data_out->set_accel_y(msg.accel_y);
  data_out->set_accel_z(msg.accel_z);
  data_out->set_pitch_rate(msg.pitch_rate);
  data_out->set_roll_rate(msg.roll_rate);
  data_out->set_yaw_rate(msg.yaw_rate);

  return true;
}

bool ParseProtoMsg::DecodeImuMessage(const Char_t* msg, Int32_t msg_len,
                                     ad_msg::Imu* data_out) {
  // parse
  msg::localization::Imu imu;
  if (!imu.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse imu from array.";
    return false;
  }

  data_out->yaw_rate = imu.yaw_rate();
  data_out->pitch_rate = imu.pitch_rate();
  data_out->roll_rate = imu.roll_rate();
  data_out->accel_x = imu.accel_x();
  data_out->accel_y = imu.accel_y();
  data_out->accel_z = imu.accel_z();

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}

bool ParseProtoMsg::EncodeRtkResultMessage(
    const ad_msg::RtkResult& msg, middle_layer_pb::RtkResult* const data_out) {
  data_out->mutable_gnss_result()->set_lat(msg.gnss_result.lat);
  data_out->mutable_gnss_result()->set_lon(msg.gnss_result.lon);
  data_out->mutable_gnss_result()->set_height(msg.gnss_result.height);
  data_out->mutable_utm_result()->set_x(msg.utm_result.x);
  data_out->mutable_utm_result()->set_y(msg.utm_result.y);
  data_out->mutable_utm_result()->set_z(msg.utm_result.z);
  return true;
}
bool ParseProtoMsg::DecodeRtkResultMessage(const Char_t* msg, Int32_t msg_len,
                                           ad_msg::RtkResult* data_out) {
  // parse
  middle_layer_pb::RtkResult rtk_res;
  if (!rtk_res.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse RtkResult from array.";
    return false;
  }
  data_out->gnss_result.lat = rtk_res.gnss_result().lat();
  data_out->gnss_result.lon = rtk_res.gnss_result().lon();
  data_out->gnss_result.height = rtk_res.gnss_result().height();
  data_out->utm_result.x = rtk_res.utm_result().x();
  data_out->utm_result.y = rtk_res.utm_result().y();
  data_out->utm_result.z = rtk_res.utm_result().z();

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}

bool ParseProtoMsg::EncodeLaserResultMessage(
    const ad_msg::LaserResult& msg,
    middle_layer_pb::LaserResult* const data_out) {
  data_out->set_height(msg.height);
  return true;
}
bool ParseProtoMsg::DecodeLaserResultMessage(const Char_t* msg, Int32_t msg_len,
                                             ad_msg::LaserResult* data_out) {
  // parse
  middle_layer_pb::LaserResult laser_res;
  if (!laser_res.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse LaserResult from array.";
    return false;
  }
  data_out->height = laser_res.height();
  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();
  return true;
}

  bool ParseProtoMsg::EncodeWritingProcessInfoMessage(const ad_msg::WritingWordInfo& msg,
                      std::string& str){
                          msg::localization::WritingProcessInfo data_out;
                          msg::localization::WritingParams* params = new msg::localization::WritingParams();
                          msg::localization::WritingSys* writingSys = new msg::localization::WritingSys();
                          params->set_offset_x(msg.offset_x);
                          params->set_offset_y(msg.offset_y);
                          params->set_add_material_status(msg.add_material_status);
                          params->set_dusting_switch_type(msg.dusting_switch_type);
                          params->set_is_move(msg.is_move);
                          writingSys->set_ack(0);
                          Int64_t timestamp = common::GetClockNowMs();
                          data_out.set_id("Test");
                          data_out.set_time(timestamp);
                          data_out.set_version("1.0");
                          data_out.set_sitecode("K11");
                          data_out.set_method("thing.event.slidepole.post");
                          data_out.set_allocated_params(params);//data_out析构 会delete params
                          data_out.set_allocated_sys(writingSys);//data_out析构 会delete writingSys
                          if(!ParseToJsonFormProto(data_out,str)){
                            LOG_ERR << "Failed to parse WritingProcessInfo from array.";
                            return false;
                          }
                          return true;
                        }
  bool ParseProtoMsg::DecodeWritingProcessInfoMessage(const std::string str,
                        ad_msg::WritingWordInfo* data_out){
                          msg::localization::WritingProcessInfo writing_process_info;
                          msg::localization::WritingParams params;
                          if (!ParesToProtoFormJson(str, writing_process_info)) {
                            LOG_ERR << "DecodeWritingProcessInfoMessage: Failed to parse WritingProcessInfo from array.";
                            return false;
                          }
                          params = writing_process_info.params();
                          data_out->offset_x = params.offset_x();
                          data_out->offset_y = params.offset_y();
                          data_out->is_move = params.is_move();
                          data_out->add_material_status = params.add_material_status();
                          data_out->dusting_switch_type = params.dusting_switch_type();
                          return true;
                        }



bool ParseProtoMsg::EncodeChassisMessage(
    const ad_msg::Chassis& msg, msg::control::Chassis* const data_out) {
  // 驾驶模式
  switch (msg.driving_mode) {
    case (ad_msg::VEH_DRIVING_MODE_MANUAL):
      data_out->set_driving_mode(msg::control::Chassis::DRIVING_MODE_MANUAL);
      break;
    case (ad_msg::VEH_DRIVING_MODE_ROBOTIC):
      data_out->set_driving_mode(msg::control::Chassis::DRIVING_MODE_ROBOTIC);
      break;
      break;
    default:
      data_out->set_driving_mode(msg::control::Chassis::DRIVING_MODE_INVALID);
      break;
  }
  // 紧急停止信号
  switch (msg.e_stop) {
    case (ad_msg::VEH_E_STOP_OFF):
      data_out->set_e_stop(msg::control::Chassis::E_STOP_OFF);
      break;
    case (ad_msg::VEH_E_STOP_ON):
      data_out->set_e_stop(msg::control::Chassis::E_STOP_ON);
      break;
    default:
      data_out->set_e_stop(msg::control::Chassis::E_STOP_INVALID);
      break;
  }

  // 方向盘控制状态
  switch (msg.eps_status) {
    case (ad_msg::VEH_EPS_STATUS_MANUAL):
      data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_MANUAL);
      break;
    case (ad_msg::VEH_EPS_STATUS_ROBOTIC):
      data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_ROBOTIC);
      break;
    case (ad_msg::VEH_EPS_STATUS_MANUAL_INTERRUPT):
      data_out->set_eps_status(
          msg::control::Chassis::EPS_STATUS_MANUAL_INTERRUPT);
      break;
    case (ad_msg::VEH_EPS_STATUS_ERROR):
      data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_ERROR);
      break;
    default:
      data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_INVALID);
      break;
  }
  // 油门系统控制状态
  switch (msg.throttle_sys_status) {
    case (ad_msg::VEH_THROTTLE_SYS_STATUS_MANUAL):
      data_out->set_throttle_sys_status(
          msg::control::Chassis::THROTTLE_SYS_STATUS_MANUAL);
      break;
    case (ad_msg::VEH_THROTTLE_SYS_STATUS_ROBOTIC):
      data_out->set_throttle_sys_status(
          msg::control::Chassis::THROTTLE_SYS_STATUS_ROBOTIC);
      break;
    case (ad_msg::VEH_THROTTLE_SYS_STATUS_ERROR):
      data_out->set_throttle_sys_status(
          msg::control::Chassis::THROTTLE_SYS_STATUS_ERROR);
      break;
    default:
      data_out->set_throttle_sys_status(
          msg::control::Chassis::THROTTLE_SYS_STATUS_INVALID);
      break;
  }
  // 制动系统控制状态
  switch (msg.ebs_status) {
    case (ad_msg::VEH_EBS_STATUS_MANUAL):
      data_out->set_ebs_status(msg::control::Chassis::EBS_STATUS_MANUAL);
      break;
    case (ad_msg::VEH_EBS_STATUS_ROBOTIC):
      data_out->set_ebs_status(msg::control::Chassis::EBS_STATUS_ROBOTIC);
      break;
    case (ad_msg::VEH_EBS_STATUS_ERROR):
      data_out->set_ebs_status(msg::control::Chassis::EBS_STATUS_ERROR);
      break;
    default:
      data_out->set_ebs_status(msg::control::Chassis::EBS_STATUS_INVALID);
      break;
  }

  // 方向盘角度有效位
  data_out->set_steering_wheel_angle_valid(msg.steering_wheel_angle_valid);
  // 方向盘角度 (rad)
  data_out->set_steering_wheel_angle(msg.steering_wheel_angle);
  // 方向盘转速有效位
  data_out->set_steering_wheel_speed_valid(msg.steering_wheel_speed_valid);
  // 方向盘转速 (rad/s)
  data_out->set_steering_wheel_speed(msg.steering_wheel_speed);
  // 制动踏板深度
  data_out->set_brake_pedal_value(msg.brake_pedal_value);
  // 油门踏板深度, PH=[百分比][0,100]
  data_out->set_acc_pedal_value(msg.acc_pedal_value);

  // 档位
  switch (msg.gear) {
    case (ad_msg::VEH_GEAR_P):
      data_out->set_gear(msg::control::Chassis::GEAR_P);
      break;
    case (ad_msg::VEH_GEAR_N):
      data_out->set_gear(msg::control::Chassis::GEAR_N);
      break;
    case (ad_msg::VEH_GEAR_R):
      data_out->set_gear(msg::control::Chassis::GEAR_R);
      break;
    case (ad_msg::VEH_GEAR_D):
      data_out->set_gear(msg::control::Chassis::GEAR_D);
      break;
    default:
      data_out->set_gear(msg::control::Chassis::GEAR_INVALID);
      break;
  }
  // 转向灯信号
  switch (msg.signal_turn_lamp) {
    case (ad_msg::VEH_TURN_LAMP_OFF):
      data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_OFF);
      break;
    case (ad_msg::VEH_TURN_LAMP_RIGHT):
      data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_RIGHT);
      break;
    case (ad_msg::VEH_TURN_LAMP_LEFT):
      data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_LEFT);
      break;
    case (ad_msg::VEH_TURN_LAMP_EMERGENCY):
      data_out->set_signal_turn_lamp(
          msg::control::Chassis::TURN_LAMP_EMERGENCY);
      break;
    default:
      data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_INVALID);
      break;
  }

  switch (msg.signal_turning_indicator) {
    case (ad_msg::VEH_TURNING_INDICATOR_NONE):
      data_out->set_signal_turning_indicator(
          msg::control::Chassis::TURNING_INDICATOR_NONE);
      break;
    case (ad_msg::VEH_TURNING_INDICATOR_LEFT):
      data_out->set_signal_turning_indicator(
          msg::control::Chassis::TURNING_INDICATOR_LEFT);
      break;
    case (ad_msg::VEH_TURNING_INDICATOR_RIGHT):
      data_out->set_signal_turning_indicator(
          msg::control::Chassis::TURNING_INDICATOR_RIGHT);
      break;
    default:
      data_out->set_signal_turning_indicator(
          msg::control::Chassis::TURNING_INDICATOR_INVALID);
      break;
  }

  // 制动灯信号, 0x0:无效，0x1:OFF, 0x2:开
  switch (msg.signal_brake_lamp) {
    case (ad_msg::VEH_LAMP_OFF):
      data_out->set_signal_brake_lamp(msg::control::Chassis::LAMP_OFF);
      break;
    case (ad_msg::VEH_LAMP_ON):
      data_out->set_signal_brake_lamp(msg::control::Chassis::LAMP_ON);
      break;
    default:
      data_out->set_signal_brake_lamp(msg::control::Chassis::LAMP_INVALID);
      break;
  }
  // 驻车系统状态, 0x0:无效，0x1:驻车解除, 0x2:驻车中
  switch (msg.epb_status) {
    case (ad_msg::VEH_EPB_STATUS_OFF):
      data_out->set_epb_status(msg::control::Chassis::EPB_STATUS_OFF);
      break;
    case (ad_msg::VEH_EPB_STATUS_ON):
      data_out->set_epb_status(msg::control::Chassis::EPB_STATUS_ON);
      break;
    default:
      data_out->set_epb_status(msg::control::Chassis::EPB_STATUS_INVALID);
      break;
  }

  // 左前轮速有效位
  data_out->set_wheel_speed_fl_valid(msg.wheel_speed_fl_valid);
  // 左前轮速(m/s)
  data_out->set_wheel_speed_fl(msg.wheel_speed_fl);
  // 右前轮速有效位
  data_out->set_wheel_speed_fr_valid(msg.wheel_speed_fr_valid);
  // 右前轮速(m/s)
  data_out->set_wheel_speed_fr(msg.wheel_speed_fr);
  // 左后轮速有效位
  data_out->set_wheel_speed_rl_valid(msg.wheel_speed_rl_valid);
  // 左后轮速(m/s)
  data_out->set_wheel_speed_rl(msg.wheel_speed_rl);
  // 右后轮速有效位
  data_out->set_wheel_speed_rr_valid(msg.wheel_speed_rr_valid);
  // 右后轮速(m/s)
  data_out->set_wheel_speed_rr(msg.wheel_speed_rr);
  // 左后2#轮速有效位
  data_out->set_wheel_speed_rl2_valid(msg.wheel_speed_rl2_valid);
  // 左后2#轮速(m/s)
  data_out->set_wheel_speed_rl2(msg.wheel_speed_rl2);
  // 右后2#轮速有效位
  data_out->set_wheel_speed_rr2_valid(msg.wheel_speed_rr2_valid);
  // 右后2#轮速(m/s)
  data_out->set_wheel_speed_rr2(msg.wheel_speed_rr2);

  // 车速有效位
  data_out->set_velocity_valid(msg.v_valid);
  // 车速(m/s)
  data_out->set_velocity(msg.v);
  // 加速度有效位
  data_out->set_acceleration_valid(msg.a_valid);
  // 加速度
  data_out->set_acceleration(msg.a);

  // Yaw Rate有效位
  data_out->set_yaw_rate_valid(msg.yaw_rate_valid);
  // Yaw Rate(rad/s), PH:[-2.0943rad/s,+2.0943rad/s]
  data_out->set_yaw_rate(msg.yaw_rate);
  // AX 有效位
  data_out->set_ax_valid(msg.ax_valid);
  // AX(m/s^2), PH:[-21.593m/s^2,+21.593m/s^2]
  data_out->set_ax(msg.ax);
  // AY 有效位
  data_out->set_ay_valid(msg.ay_valid);
  // AY(m/s^2), PH:[-21.593m/s^2,+21.593m/s^2]
  data_out->set_ay(msg.ay);

  // 发动机转速有效位
  data_out->set_engine_speed_valid(msg.engine_speed_valid);
  // 发动机转速
  data_out->set_engine_speed(msg.engine_speed);
  // 发动机转矩有效位
  data_out->set_engine_torque_valid(msg.engine_torque_valid);
  // 发动机转矩(N.m)
  data_out->set_engine_torque(msg.engine_torque);

  return true;
}

bool ParseProtoMsg::DecodeChassisMessage(const Char_t* msg, Int32_t msg_len,
                                         ad_msg::Chassis* data_out) {
  // parse
  msg::control::Chassis chassis;
  if (!chassis.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse chassis from array.";
    return false;
  }

  // 驾驶模式
  switch (chassis.driving_mode()) {
    case (msg::control::Chassis::DRIVING_MODE_MANUAL):
      data_out->driving_mode = ad_msg::VEH_DRIVING_MODE_MANUAL;
      break;
    case (msg::control::Chassis::DRIVING_MODE_ROBOTIC):
      data_out->driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
      break;
    default:
      data_out->driving_mode = ad_msg::VEH_DRIVING_MODE_INVALID;
      break;
  }

  // 紧急停止信号
  switch (chassis.e_stop()) {
    case (msg::control::Chassis::E_STOP_OFF):
      data_out->e_stop = ad_msg::VEH_E_STOP_OFF;
      break;
    case (msg::control::Chassis::E_STOP_ON):
      data_out->e_stop = ad_msg::VEH_E_STOP_ON;
      break;
    default:
      data_out->e_stop = ad_msg::VEH_E_STOP_INVALID;
      break;
  }

  // 方向盘控制状态
  switch (chassis.eps_status()) {
    case (msg::control::Chassis::EPS_STATUS_MANUAL):
      data_out->eps_status = ad_msg::VEH_EPS_STATUS_MANUAL;
      break;
    case (msg::control::Chassis::EPS_STATUS_ROBOTIC):
      data_out->eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
      break;
    case (msg::control::Chassis::EPS_STATUS_MANUAL_INTERRUPT):
      data_out->eps_status = ad_msg::VEH_EPS_STATUS_MANUAL_INTERRUPT;
      break;
    case (msg::control::Chassis::EPS_STATUS_ERROR):
      data_out->eps_status = ad_msg::VEH_EPS_STATUS_ERROR;
      break;
    default:
      data_out->eps_status = ad_msg::VEH_EPS_STATUS_INVALID;
  }
  // 油门系统控制状态
  switch (chassis.throttle_sys_status()) {
    case (msg::control::Chassis::THROTTLE_SYS_STATUS_MANUAL):
      data_out->throttle_sys_status = ad_msg::VEH_THROTTLE_SYS_STATUS_MANUAL;
      break;
    case (msg::control::Chassis::THROTTLE_SYS_STATUS_ROBOTIC):
      data_out->throttle_sys_status = ad_msg::VEH_THROTTLE_SYS_STATUS_ROBOTIC;
      break;
    case (msg::control::Chassis::THROTTLE_SYS_STATUS_ERROR):
      data_out->throttle_sys_status = ad_msg::VEH_THROTTLE_SYS_STATUS_ERROR;
      break;
    default:
      data_out->throttle_sys_status = ad_msg::VEH_THROTTLE_SYS_STATUS_INVALID;
  }
  // 制动系统控制状态
  switch (chassis.ebs_status()) {
    case (msg::control::Chassis::EBS_STATUS_MANUAL):
      data_out->ebs_status = ad_msg::VEH_EBS_STATUS_MANUAL;
      break;
    case (msg::control::Chassis::EBS_STATUS_ROBOTIC):
      data_out->ebs_status = ad_msg::VEH_EBS_STATUS_ROBOTIC;
      break;
    case (msg::control::Chassis::EBS_STATUS_ERROR):
      data_out->ebs_status = ad_msg::VEH_EBS_STATUS_ERROR;
      break;
    default:
      data_out->ebs_status = ad_msg::VEH_EBS_STATUS_INVALID;
  }

  // 方向盘角度有效位
  data_out->steering_wheel_angle_valid = chassis.steering_wheel_angle_valid();
  // 方向盘角度 (rad)
  data_out->steering_wheel_angle = chassis.steering_wheel_angle();
  // 方向盘转速有效位
  data_out->steering_wheel_speed_valid = chassis.steering_wheel_speed_valid();
  // 方向盘转速 (rad/s)
  data_out->steering_wheel_speed = chassis.steering_wheel_speed();
  // 实际转向扭矩有效位
  data_out->steering_wheel_torque_valid = chassis.steering_wheel_torque_valid();
  // 实际转向扭矩(N.m)
  data_out->steering_wheel_torque = chassis.steering_wheel_torque();

  // 车速有效位
  data_out->v_valid = chassis.velocity_valid();
  // 车速(m/s)
  data_out->v = chassis.velocity();
  // 加速度有效位
  data_out->a_valid = chassis.acceleration_valid();
  // 加速度
  data_out->a = chassis.acceleration();
  // Yaw Rate有效位
  data_out->yaw_rate_valid = chassis.yaw_rate_valid();
  // Yaw Rate(rad/s), PH:[-2.0943rad/s,+2.0943rad/s]
  data_out->yaw_rate = chassis.yaw_rate();
  // AX 有效位
  data_out->ax_valid = chassis.ax_valid();
  // AX(m/s^2), PH:[-21.593m/s^2,+21.593m/s^2]
  data_out->ax = chassis.ax();
  // AY 有效位
  data_out->ay_valid = chassis.ay_valid();
  // AY(m/s^2), PH:[-21.593m/s^2,+21.593m/s^2]
  data_out->ay = chassis.ay();

  // 左前轮速有效位
  data_out->wheel_speed_fl_valid = chassis.wheel_speed_fl_valid();
  // 左前轮速(m/s)
  data_out->wheel_speed_fl = chassis.wheel_speed_fl();
  // 右前轮速有效位
  data_out->wheel_speed_fr_valid = chassis.wheel_speed_fr_valid();
  // 右前轮速(m/s)
  data_out->wheel_speed_fr = chassis.wheel_speed_fr();
  // 左后轮速有效位
  data_out->wheel_speed_rl_valid = chassis.wheel_speed_rl_valid();
  // 左后轮速(m/s)
  data_out->wheel_speed_rl = chassis.wheel_speed_rl();
  // 右后轮速有效位
  data_out->wheel_speed_rr_valid = chassis.wheel_speed_rr_valid();
  // 右后轮速(m/s)
  data_out->wheel_speed_rr = chassis.wheel_speed_rr();
  // 左后2#轮速有效位
  data_out->wheel_speed_rl2_valid = chassis.wheel_speed_rl2_valid();
  // 左后2#轮速(m/s)
  data_out->wheel_speed_rl2 = chassis.wheel_speed_rl2();
  // 右后2#轮速有效位
  data_out->wheel_speed_rr2_valid = chassis.wheel_speed_rr2_valid();
  // 右后2#轮速(m/s)
  data_out->wheel_speed_rr2 = chassis.wheel_speed_rr2();

  // 驻车系统状态
  switch (chassis.epb_status()) {
    case (msg::control::Chassis::EPB_STATUS_OFF):
      data_out->epb_status = ad_msg::VEH_EPB_STATUS_OFF;
      break;
    case (msg::control::Chassis::EPB_STATUS_ON):
      data_out->epb_status = ad_msg::VEH_EPB_STATUS_ON;
      break;
    default:
      data_out->epb_status = ad_msg::VEH_EPB_STATUS_INVALID;
      break;
  }
  // 档位
  switch (chassis.gear()) {
    case (msg::control::Chassis::GEAR_P):
      data_out->gear = ad_msg::VEH_GEAR_P;
      break;
    case (msg::control::Chassis::GEAR_N):
      data_out->gear = ad_msg::VEH_GEAR_N;
      break;
    case (msg::control::Chassis::GEAR_R):
      data_out->gear = ad_msg::VEH_GEAR_R;
      break;
    case (msg::control::Chassis::GEAR_D):
      data_out->gear = ad_msg::VEH_GEAR_D;
      break;
    default:
      data_out->gear = ad_msg::VEH_GEAR_INVALID;
      break;
  }
  // 档位 Number
  data_out->gear_number = chassis.gear_number();
  // 转向拨杆信号
  switch (chassis.signal_turning_indicator()) {
    case (msg::control::Chassis::TURNING_INDICATOR_NONE):
      data_out->signal_turning_indicator = ad_msg::VEH_TURNING_INDICATOR_NONE;
      break;
    case (msg::control::Chassis::TURNING_INDICATOR_LEFT):
      data_out->signal_turning_indicator = ad_msg::VEH_TURNING_INDICATOR_LEFT;
      break;
    case (msg::control::Chassis::TURNING_INDICATOR_RIGHT):
      data_out->signal_turning_indicator = ad_msg::VEH_TURNING_INDICATOR_RIGHT;
      break;
    default:
      data_out->signal_turning_indicator =
          ad_msg::VEH_TURNING_INDICATOR_INVALID;
      break;
  }
  // 转向灯信号
  switch (chassis.signal_turn_lamp()) {
    case (msg::control::Chassis::TURN_LAMP_OFF):
      data_out->signal_turn_lamp = ad_msg::VEH_TURN_LAMP_OFF;
      break;
    case (msg::control::Chassis::TURN_LAMP_LEFT):
      data_out->signal_turn_lamp = ad_msg::VEH_TURN_LAMP_LEFT;
      break;
    case (msg::control::Chassis::TURN_LAMP_RIGHT):
      data_out->signal_turn_lamp = ad_msg::VEH_TURN_LAMP_RIGHT;
      break;
    case (msg::control::Chassis::TURN_LAMP_EMERGENCY):
      data_out->signal_turn_lamp = ad_msg::VEH_TURN_LAMP_EMERGENCY;
      break;
    default:
      data_out->signal_turn_lamp = ad_msg::VEH_TURN_LAMP_INVALID;
      break;
  }
  // 制动灯信号
  switch (chassis.signal_brake_lamp()) {
    case (msg::control::Chassis::LAMP_OFF):
      data_out->signal_brake_lamp = ad_msg::VEH_LAMP_OFF;
      break;
    case (msg::control::Chassis::LAMP_ON):
      data_out->signal_brake_lamp = ad_msg::VEH_LAMP_ON;
      break;
    default:
      data_out->signal_brake_lamp = ad_msg::VEH_LAMP_INVALID;
      break;
  }
  // 制动踏板深度, PH=[百分比][0,100]
  data_out->brake_pedal_value = chassis.brake_pedal_value();
  // 油门踏板深度, PH=[百分比][0,100]
  data_out->acc_pedal_value = chassis.acc_pedal_value();

  // 发动机转速有效位
  data_out->engine_speed_valid = chassis.engine_speed_valid();
  // 发动机转速
  data_out->engine_speed = chassis.engine_speed();
  // 发动机转矩有效位
  data_out->engine_torque_valid = chassis.engine_torque_valid();
  // 发动机转矩(N.m)
  data_out->engine_torque = chassis.engine_torque();

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}

bool ParseProtoMsg::EncodeSpecialChassisInfoMessage(
    const ad_msg::SpecialChassisInfo& msg,
    msg::control::SpecialChassisInfo* const data_out) {
  // 开始自动驾驶命令, 0 ~ 无效值, 1 ~ 退出自动驾驶, 2 ~ 进入自动驾驶
  data_out->set_start_adas(msg.start_adas);

  // CAN线连接状态：0 ~ 正常, 1 ~ 丢帧
  data_out->set_cnt_stu_frame_loss_can0(msg.cnt_stu_frame_loss_can0);
  data_out->set_cnt_stu_frame_loss_can1(msg.cnt_stu_frame_loss_can1);
  data_out->set_cnt_stu_frame_loss_can2(msg.cnt_stu_frame_loss_can2);
  data_out->set_cnt_stu_frame_loss_can3(msg.cnt_stu_frame_loss_can3);

  // 网关和车身的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  data_out->set_cnt_stu_gtw_to_veh_can0(msg.cnt_stu_gtw_to_veh_can0);
  data_out->set_cnt_stu_gtw_to_veh_can1(msg.cnt_stu_gtw_to_veh_can1);
  data_out->set_cnt_stu_gtw_to_veh_can2(msg.cnt_stu_gtw_to_veh_can2);
  data_out->set_cnt_stu_gtw_to_veh_can3(msg.cnt_stu_gtw_to_veh_can3);

  // 控制侧和网关的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  data_out->set_cnt_stu_ctl_to_gtw_can0(msg.cnt_stu_ctl_to_gtw_can0);
  data_out->set_cnt_stu_ctl_to_gtw_can1(msg.cnt_stu_ctl_to_gtw_can1);
  data_out->set_cnt_stu_ctl_to_gtw_can2(msg.cnt_stu_ctl_to_gtw_can2);
  data_out->set_cnt_stu_ctl_to_gtw_can3(msg.cnt_stu_ctl_to_gtw_can3);

  // 网关和控制侧的连接状态: 0 ~ 未连接, 1 ~ 已连接
  data_out->set_cnt_stu_ctl_to_gtw(msg.cnt_stu_ctl_to_gtw);

  // 具体车型的特殊信息 (FT-Auman)
  msg::control::ChassisFtAuman* ft_auman = data_out->mutable_ft_auman();
  ft_auman->set_switch_tja(msg.ft_auman.switch_tja);
  ft_auman->set_switch_hwa(msg.ft_auman.switch_hwa);
  ft_auman->set_switch_i_drive(msg.ft_auman.switch_i_drive);

  return (true);
}

bool ParseProtoMsg::DecodeSpecialChassisInfoMessage(
    const Char_t* msg, Int32_t msg_len, ad_msg::SpecialChassisInfo* data_out) {
  // parse
  msg::control::SpecialChassisInfo special_chassis_info;
  if (!special_chassis_info.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse chassis from array.";
    return false;
  }

  // 开始自动驾驶命令, 0 ~ 无效值, 1 ~ 退出自动驾驶, 2 ~ 进入自动驾驶
  data_out->start_adas = special_chassis_info.start_adas();

  // CAN线连接状态：0 ~ 正常, 1 ~ 丢帧
  data_out->cnt_stu_frame_loss_can0 =
      special_chassis_info.cnt_stu_frame_loss_can0();
  data_out->cnt_stu_frame_loss_can1 =
      special_chassis_info.cnt_stu_frame_loss_can1();
  data_out->cnt_stu_frame_loss_can2 =
      special_chassis_info.cnt_stu_frame_loss_can2();
  data_out->cnt_stu_frame_loss_can3 =
      special_chassis_info.cnt_stu_frame_loss_can3();

  // 网关和车身的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  data_out->cnt_stu_gtw_to_veh_can0 =
      special_chassis_info.cnt_stu_gtw_to_veh_can0();
  data_out->cnt_stu_gtw_to_veh_can1 =
      special_chassis_info.cnt_stu_gtw_to_veh_can1();
  data_out->cnt_stu_gtw_to_veh_can2 =
      special_chassis_info.cnt_stu_gtw_to_veh_can2();
  data_out->cnt_stu_gtw_to_veh_can3 =
      special_chassis_info.cnt_stu_gtw_to_veh_can3();

  // 控制侧和网关的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  data_out->cnt_stu_ctl_to_gtw_can0 =
      special_chassis_info.cnt_stu_ctl_to_gtw_can0();
  data_out->cnt_stu_ctl_to_gtw_can1 =
      special_chassis_info.cnt_stu_ctl_to_gtw_can1();
  data_out->cnt_stu_ctl_to_gtw_can2 =
      special_chassis_info.cnt_stu_ctl_to_gtw_can2();
  data_out->cnt_stu_ctl_to_gtw_can3 =
      special_chassis_info.cnt_stu_ctl_to_gtw_can3();

  // 网关和控制侧的连接状态: 0 ~ 未连接, 1 ~ 已连接
  data_out->cnt_stu_ctl_to_gtw = special_chassis_info.cnt_stu_ctl_to_gtw();

  // 具体车型的特殊信息 (FT-Auman)
  const msg::control::ChassisFtAuman& ft_auman =
      special_chassis_info.ft_auman();
  data_out->ft_auman.switch_tja = ft_auman.switch_tja();
  data_out->ft_auman.switch_hwa = ft_auman.switch_hwa();
  data_out->ft_auman.switch_i_drive = ft_auman.switch_i_drive();

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}

bool ParseProtoMsg::EncodeChassisCtlCmdMessage(
    const ad_msg::ChassisCtlCmd& msg,
    msg::control::ChassisCtlCmd* const data_out) {
  // 开始自动驾驶命令
  data_out->set_start_robotic_ctl(msg.start_robotic_ctl);
  // 使能转向控制系统
  data_out->set_enable_eps(msg.enable_eps);
  // 使能油门控制系统
  data_out->set_enable_throttle_sys(msg.enable_throttle_sys);
  // 使能制动控制系统
  data_out->set_enable_ebs(msg.enable_ebs);

  // 使能远程控制
  data_out->set_enable_remote_ctl(msg.enable_remote_ctl);
  // 使能直接控制模式
  data_out->set_enable_direct_ctl(msg.enable_direct_ctl);
  // 使能速度控制
  data_out->set_enable_acc(msg.enable_acc);
  // 释放油门控制
  data_out->set_release_throttle(msg.release_throttle);

  // 方向盘角度 (rad)
  data_out->set_steering_wheel_angle(msg.steering_wheel_angle);
  // 方向盘转速 (rad/s)
  data_out->set_steering_wheel_speed(msg.steering_wheel_speed);
  // 车速(m/s)
  data_out->set_velocity(msg.velocity);
  // 加速度
  data_out->set_acceleration(msg.acceleration);
  // 加速度量
  data_out->set_acc_value(msg.acc_value);
  // 刹车量
  data_out->set_brake_value(msg.brake_value);

  // 档位
  switch (msg.gear) {
    case (ad_msg::VEH_GEAR_P):
      data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_P);
      break;
    case (ad_msg::VEH_GEAR_N):
      data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_N);
      break;
    case (ad_msg::VEH_GEAR_R):
      data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_R);
      break;
    case (ad_msg::VEH_GEAR_D):
      data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_D);
      break;
    default:
      data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_INVALID);
      break;
  }
  // 转向灯信号
  switch (msg.turn_lamp) {
    case (ad_msg::VEH_TURN_LAMP_OFF):
      data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_OFF);
      break;
    case (ad_msg::VEH_TURN_LAMP_RIGHT):
      data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_RIGHT);
      break;
    case (ad_msg::VEH_TURN_LAMP_LEFT):
      data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_LEFT);
      break;
    case (ad_msg::VEH_TURN_LAMP_EMERGENCY):
      data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_EMERGENCY);
      break;
    default:
      data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_INVALID);
      break;
  }

  // 制动灯信号, 0x0:无效，0x1:OFF, 0x2:开
  switch (msg.brake_lamp) {
    case (ad_msg::VEH_LAMP_OFF):
      data_out->set_brake_lamp(msg::control::ChassisCtlCmd::LAMP_OFF);
      break;
    case (ad_msg::VEH_LAMP_ON):
      data_out->set_brake_lamp(msg::control::ChassisCtlCmd::LAMP_ON);
      break;
    default:
      data_out->set_brake_lamp(msg::control::ChassisCtlCmd::LAMP_INVALID);
      break;
  }

  // 驻车系统状态, 0x0:无效，0x1:驻车解除, 0x2:驻车中
  switch (msg.epb_status) {
    case (ad_msg::VEH_EPB_STATUS_OFF):
      data_out->set_epb_status(msg::control::ChassisCtlCmd::EPB_STATUS_OFF);
      break;
    case (ad_msg::VEH_EPB_STATUS_ON):
      data_out->set_epb_status(msg::control::ChassisCtlCmd::EPB_STATUS_ON);
      break;
    default:
      data_out->set_epb_status(msg::control::ChassisCtlCmd::EPB_STATUS_INVALID);
      break;
  }

  // 雨刮器状态, 0x0:无效，0x1:关闭, 0x2:SHORT PRESS WITH CLICK,
  // 0x3:LONG PRESS WITH CLICK，0x4:INT 1, 0x5:INT 2，0x6:INT 3,
  // 0x7:INT 4，0x8:LO，0x9:HI
  switch (msg.wiper) {
    case (ad_msg::VEH_WIPER_OFF):
      data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_OFF);
      break;
    case (ad_msg::VEH_WIPER_SHORT_PRESS_WITH_CLICK):
      data_out->set_wiper(
          msg::control::ChassisCtlCmd::WIPER_SHORT_PRESS_WITH_CLICK);
      break;
    case (ad_msg::VEH_WIPER_LONG_PRESS_WITH_CLICK):
      data_out->set_wiper(
          msg::control::ChassisCtlCmd::WIPER_LONG_PRESS_WITH_CLICK);
      break;
    case (ad_msg::VEH_WIPER_INT_1):
      data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INT_1);
      break;
    case (ad_msg::VEH_WIPER_INT_2):
      data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INT_2);
      break;
    case (ad_msg::VEH_WIPER_INT_3):
      data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INT_3);
      break;
    case (ad_msg::VEH_WIPER_INT_4):
      data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INT_4);
      break;
    case (ad_msg::VEH_WIPER_LO):
      data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_LO);
      break;
    case (ad_msg::VEH_WIPER_HI):
      data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_HI);
      break;
    default:
      data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INVALID);
      break;
  }

  return true;
}

bool ParseProtoMsg::DecodeChassisCtlCmdMessage(
    const Char_t* msg, Int32_t msg_len, ad_msg::ChassisCtlCmd* data_out) {
  // parse
  msg::control::ChassisCtlCmd chassis_ctl_cmd;
  if (!chassis_ctl_cmd.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse chassis from array.";
    return false;
  }

  // 开始自动驾驶命令
  data_out->start_robotic_ctl = chassis_ctl_cmd.start_robotic_ctl();
  // 使能转向控制系统
  data_out->enable_eps = chassis_ctl_cmd.enable_eps();
  // 使能油门控制系统
  data_out->enable_throttle_sys = chassis_ctl_cmd.enable_throttle_sys();
  // 使能制动控制系统
  data_out->enable_ebs = chassis_ctl_cmd.enable_ebs();

  // 使能远程控制
  data_out->enable_remote_ctl = chassis_ctl_cmd.enable_remote_ctl();
  // 使能直接控制模式
  data_out->enable_direct_ctl = chassis_ctl_cmd.enable_direct_ctl();
  // 使能速度控制
  data_out->enable_acc = chassis_ctl_cmd.enable_acc();
  // 释放油门控制
  data_out->release_throttle = chassis_ctl_cmd.release_throttle();

  // 方向盘角度 (rad)
  data_out->steering_wheel_angle = chassis_ctl_cmd.steering_wheel_angle();
  // 方向盘转速 (rad/s)
  data_out->steering_wheel_speed = chassis_ctl_cmd.steering_wheel_speed();
  // 车速(m/s)
  data_out->velocity = chassis_ctl_cmd.velocity();
  // 加速度
  data_out->acceleration = chassis_ctl_cmd.acceleration();
  // 加速度量
  data_out->acc_value = chassis_ctl_cmd.acc_value();
  // 刹车量
  data_out->brake_value = chassis_ctl_cmd.brake_value();

  // 档位
  switch (chassis_ctl_cmd.gear()) {
    case (msg::control::ChassisCtlCmd::GEAR_P):
      data_out->gear = ad_msg::VEH_GEAR_P;
      break;
    case (msg::control::ChassisCtlCmd::GEAR_N):
      data_out->gear = ad_msg::VEH_GEAR_N;
      break;
    case (msg::control::ChassisCtlCmd::GEAR_R):
      data_out->gear = ad_msg::VEH_GEAR_R;
      break;
    case (msg::control::ChassisCtlCmd::GEAR_D):
      data_out->gear = ad_msg::VEH_GEAR_D;
      break;
    default:
      data_out->gear = ad_msg::VEH_GEAR_INVALID;
      break;
  }
  // 转向灯信号
  switch (chassis_ctl_cmd.turn_lamp()) {
    case (msg::control::ChassisCtlCmd::TURN_LAMP_OFF):
      data_out->turn_lamp = ad_msg::VEH_TURN_LAMP_OFF;
      break;
    case (msg::control::ChassisCtlCmd::TURN_LAMP_LEFT):
      data_out->turn_lamp = ad_msg::VEH_TURN_LAMP_LEFT;
      break;
    case (msg::control::ChassisCtlCmd::TURN_LAMP_RIGHT):
      data_out->turn_lamp = ad_msg::VEH_TURN_LAMP_RIGHT;
      break;
    case (msg::control::ChassisCtlCmd::TURN_LAMP_EMERGENCY):
      data_out->turn_lamp = ad_msg::VEH_TURN_LAMP_EMERGENCY;
      break;
    default:
      data_out->turn_lamp = ad_msg::VEH_TURN_LAMP_INVALID;
      break;
  }
  // 制动灯信号, 0x0:无效，0x1:OFF, 0x2:开
  switch (chassis_ctl_cmd.brake_lamp()) {
    case (msg::control::ChassisCtlCmd::LAMP_OFF):
      data_out->brake_lamp = ad_msg::VEH_LAMP_OFF;
      break;
    case (msg::control::ChassisCtlCmd::LAMP_ON):
      data_out->brake_lamp = ad_msg::VEH_LAMP_ON;
      break;
    default:
      data_out->brake_lamp = ad_msg::VEH_LAMP_INVALID;
      break;
  }
  // 驻车系统状态, 0x0:无效，0x1:驻车解除, 0x2:驻车中
  switch (chassis_ctl_cmd.epb_status()) {
    case (msg::control::ChassisCtlCmd::EPB_STATUS_OFF):
      data_out->epb_status = ad_msg::VEH_EPB_STATUS_OFF;
      break;
    case (msg::control::ChassisCtlCmd::EPB_STATUS_ON):
      data_out->epb_status = ad_msg::VEH_EPB_STATUS_ON;
      break;
    default:
      data_out->epb_status = ad_msg::VEH_EPB_STATUS_INVALID;
      break;
  }
  // 雨刮器状态, 0x0:无效，0x1:关闭, 0x2:SHORT PRESS WITH CLICK,
  // 0x3:LONG PRESS WITH CLICK，0x4:INT 1, 0x5:INT 2，0x6:INT 3,
  // 0x7:INT 4，0x8:LO，0x9:HI
  switch (chassis_ctl_cmd.wiper()) {
    case (msg::control::ChassisCtlCmd::WIPER_OFF):
      data_out->wiper = ad_msg::VEH_WIPER_OFF;
      break;
    case (msg::control::ChassisCtlCmd::WIPER_SHORT_PRESS_WITH_CLICK):
      data_out->wiper = ad_msg::VEH_WIPER_SHORT_PRESS_WITH_CLICK;
      break;
    case (msg::control::ChassisCtlCmd::WIPER_LONG_PRESS_WITH_CLICK):
      data_out->wiper = ad_msg::VEH_WIPER_LONG_PRESS_WITH_CLICK;
      break;
    case (msg::control::ChassisCtlCmd::WIPER_INT_1):
      data_out->wiper = ad_msg::VEH_WIPER_INT_1;
      break;
    case (msg::control::ChassisCtlCmd::WIPER_INT_2):
      data_out->wiper = ad_msg::VEH_WIPER_INT_2;
      break;
    case (msg::control::ChassisCtlCmd::WIPER_INT_3):
      data_out->wiper = ad_msg::VEH_WIPER_INT_3;
      break;
    case (msg::control::ChassisCtlCmd::WIPER_INT_4):
      data_out->wiper = ad_msg::VEH_WIPER_INT_4;
      break;
    case (msg::control::ChassisCtlCmd::WIPER_LO):
      data_out->wiper = ad_msg::VEH_WIPER_LO;
      break;
    case (msg::control::ChassisCtlCmd::WIPER_HI):
      data_out->wiper = ad_msg::VEH_WIPER_HI;
      break;
    default:
      data_out->wiper = ad_msg::VEH_WIPER_INVALID;
      break;
  }

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}

bool ParseProtoMsg::DecodeSceneStoryControlLine(
    const msg::routing::ControlLine& line_in,
    ad_msg::SceneStoryControlLine* line_out) {
  line_out->start_point.set_x(line_in.start_point().x());
  line_out->start_point.set_y(line_in.start_point().y());
  line_out->end_point.set_x(line_in.end_point().x());
  line_out->end_point.set_y(line_in.end_point().y());

  return true;
}

bool ParseProtoMsg::DecodeSceneStoryCondition(
    const msg::routing::Condition& cond_in,
    ad_msg::SceneStoryCondition* cond_out) {
  if (cond_in.has_start_s() && cond_in.has_end_s()) {
    cond_out->valid_s = 1;
    cond_out->start_s = cond_in.start_s();
    cond_out->end_s = cond_in.end_s();
  } else {
    cond_out->valid_s = 0;
    cond_out->start_s = 0.0F;
    cond_out->end_s = 0.0F;
  }

  if (cond_in.has_speed_high()) {
    cond_out->valid_speed_high = 1;
    cond_out->speed_high = cond_in.speed_high();
  } else {
    cond_out->valid_speed_high = 0;
    cond_out->speed_high = 0.0F;
  }
  if (cond_in.has_speed_low()) {
    cond_out->valid_speed_low = 1;
    cond_out->speed_low = cond_in.speed_low();
  } else {
    cond_out->valid_speed_low = 0;
    cond_out->speed_low = 0.0F;
  }

  /// TODO: T.B.D.
  cond_out->gear = ad_msg::VEH_GEAR_INVALID;

  return true;
}

bool ParseProtoMsg::DecodeSceneStoryAction(
    const msg::routing::Action& action_in,
    ad_msg::SceneStoryAction* action_out) {
  if (action_in.has_run_time()) {
    action_out->holding_time = action_in.run_time() * 1000;
  } else {
    action_out->holding_time = -1;
  }

  if (action_in.has_speed()) {
    action_out->valid_speed = 1;
    action_out->speed = action_in.speed();
  } else {
    action_out->valid_speed = 0;
    action_out->speed = 0.0F;
  }

  if (action_in.has_acceleration()) {
    action_out->valid_acceleration = 1;
    action_out->acceleration = action_in.acceleration();
  } else {
    action_out->valid_acceleration = 0;
    action_out->acceleration = 0.0F;
  }

  if (action_in.has_gear()) {
    switch (action_in.gear()) {
      case (msg::control::Chassis::GEAR_P):
        action_out->gear = ad_msg::VEH_GEAR_P;
        break;
      case (msg::control::Chassis::GEAR_N):
        action_out->gear = ad_msg::VEH_GEAR_N;
        break;
      case (msg::control::Chassis::GEAR_R):
        action_out->gear = ad_msg::VEH_GEAR_R;
        break;
      case (msg::control::Chassis::GEAR_D):
        action_out->gear = ad_msg::VEH_GEAR_D;
        break;
      default:
        action_out->gear = ad_msg::VEH_GEAR_INVALID;
        break;
    }
  } else {
    action_out->gear = ad_msg::VEH_GEAR_INVALID;
  }

  if (action_in.has_turn_lamp()) {
    switch (action_in.turn_lamp()) {
      case (msg::control::Chassis::TURN_LAMP_OFF):
        action_out->turn_lamp = ad_msg::VEH_TURN_LAMP_OFF;
        break;
      case (msg::control::Chassis::TURN_LAMP_LEFT):
        action_out->turn_lamp = ad_msg::VEH_TURN_LAMP_LEFT;
        break;
      case (msg::control::Chassis::TURN_LAMP_RIGHT):
        action_out->turn_lamp = ad_msg::VEH_TURN_LAMP_RIGHT;
        break;
      case (msg::control::Chassis::TURN_LAMP_EMERGENCY):
        action_out->turn_lamp = ad_msg::VEH_TURN_LAMP_EMERGENCY;
        break;
      default:
        action_out->turn_lamp = ad_msg::VEH_TURN_LAMP_INVALID;
        break;
    }
  } else {
    action_out->turn_lamp = ad_msg::VEH_TURN_LAMP_INVALID;
  }

  if (action_in.has_brake_lamp()) {
    switch (action_in.brake_lamp()) {
      case (msg::control::Chassis::LAMP_OFF):
        action_out->turn_lamp = ad_msg::VEH_LAMP_OFF;
        break;
      case (msg::control::Chassis::LAMP_ON):
        action_out->turn_lamp = ad_msg::VEH_LAMP_ON;
        break;
      default:
        action_out->turn_lamp = ad_msg::VEH_LAMP_INVALID;
        break;
    }
  } else {
    action_out->turn_lamp = ad_msg::VEH_LAMP_INVALID;
  }

  return true;
}

bool ParseProtoMsg::DecodeSceneStorysMessage(const Char_t* msg, Int32_t msg_len,
                                             ad_msg::SceneStoryList* data_out) {
  // parse
  msg::routing::Scene_Stories scene_storys;
  if (!scene_storys.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse scene_storys from array.";
    return false;
  }

  Int32_t story_list_idx = 0;
  Int32_t story_num = 0;
  // 接近终点
  Int32_t story_size = scene_storys.close_to_destination_size();
  for (Int32_t i = 0; i < story_size; ++i) {
    if (story_list_idx >= ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
      break;
    }
    ad_msg::SceneStory& story = data_out->storys[story_list_idx];
    story_list_idx++;

    const msg::routing::CloseToDestination data_in =
        scene_storys.close_to_destination(i);

    story.id = data_in.id();
    story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_DESTINATION;
    story.distance = data_in.distance();

    DecodeSceneStoryControlLine(data_in.control_line(), &(story.control_line));

    if (data_in.has_condition()) {
      story.condition.vaild = true;
      DecodeSceneStoryCondition(data_in.condition(), &(story.condition));
    } else {
      story.condition.vaild = false;
    }

    if (data_in.has_action()) {
      story.action.vaild = true;
      DecodeSceneStoryAction(data_in.action(), &(story.action));
    } else {
      story.action.vaild = false;
    }
  }

  // 接近信号灯
  story_size = scene_storys.close_to_signal().size();
  for (Int32_t i = 0; i < story_size; ++i) {
    if (story_list_idx >= ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
      break;
    }
    ad_msg::SceneStory& story = data_out->storys[story_list_idx];
    story_list_idx++;

    const msg::routing::CloseToSignal data_in = scene_storys.close_to_signal(i);
    story.id = data_in.id();
    story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_TRAFFIC_SIGNAL;
    story.distance = data_in.distance();

    DecodeSceneStoryControlLine(data_in.control_line(), &(story.control_line));

    if (data_in.has_condition()) {
      story.condition.vaild = true;
      DecodeSceneStoryCondition(data_in.condition(), &(story.condition));
    } else {
      story.condition.vaild = false;
    }

    if (data_in.has_action()) {
      story.action.vaild = true;
      DecodeSceneStoryAction(data_in.action(), &(story.action));
    } else {
      story.action.vaild = false;
    }
  }

  // 接近弯道
  story_size = scene_storys.close_to_curve_road_size();
  for (Int32_t i = 0; i < story_size; ++i) {
    if (story_list_idx >= ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
      break;
    }
    ad_msg::SceneStory& story = data_out->storys[story_list_idx];
    story_list_idx++;

    const msg::routing::CloseToCurveRoad data_in =
        scene_storys.close_to_curve_road(i);
    story.id = data_in.id();
    story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_CURVE;
    story.distance = data_in.distance();

    DecodeSceneStoryControlLine(data_in.control_line(), &(story.control_line));

    if (data_in.has_condition()) {
      story.condition.vaild = true;
      DecodeSceneStoryCondition(data_in.condition(), &(story.condition));
    } else {
      story.condition.vaild = false;
    }

    if (data_in.has_action()) {
      story.action.vaild = true;
      DecodeSceneStoryAction(data_in.action(), &(story.action));
    } else {
      story.action.vaild = false;
    }
  }
  // longjiaoy
  story_size = scene_storys.close_to_crosswalk_size();
  for (Int32_t i = 0; i < story_size; ++i) {
    if (story_list_idx >= ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
      break;
    }
    ad_msg::SceneStory& story = data_out->storys[story_list_idx];
    story_list_idx++;

    const msg::routing::CloseToCrosswalk data_in =
        scene_storys.close_to_crosswalk(i);
    story.id = data_in.id();
    story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_CROSSWALK;
    story.distance = data_in.distance();

    DecodeSceneStoryControlLine(data_in.control_line(), &(story.control_line));

    if (data_in.has_condition()) {
      story.condition.vaild = true;
      DecodeSceneStoryCondition(data_in.condition(), &(story.condition));
    } else {
      story.condition.vaild = false;
    }

    if (data_in.has_action()) {
      story.action.vaild = true;
      DecodeSceneStoryAction(data_in.action(), &(story.action));
    } else {
      story.action.vaild = false;
    }
  }
  // lonjiaoy end

  data_out->story_num = story_list_idx;

  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return (true);
}

bool ParseProtoMsg::DecodeMapLocalizationMessage(
    const Char_t* msg, Int32_t msg_len, ad_msg::MapLocalization* data_out) {
  // parse
  msg::routing::MapLocalization map_localization;
  if (!map_localization.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse scene_storys from array.";
    return false;
  }

  data_out->curr_lane_id = map_localization.point().nearest_lane_id();
  data_out->s = map_localization.point().s();
  data_out->l = map_localization.point().l();
  data_out->heading = map_localization.point().heading();

  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return (true);
}

}  // namespace framework
}  // namespace phoenix
