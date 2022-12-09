#include "communication/msg_server.h"

#include <thread>

#include "communication/msg_sender.h"
#include "communication/svg_path.h"
#include "communication/svg_transform.h"
#include "pc/util.h"
#include "sensor/servo/cam_path_parse.h"
#include "utils/gps_tools.h"

#define DEBUG_SERVO (1)
#define WORD_WIDE_VALUE (0.10)

#define HEIGHT_OFFSET_VALUE (0.0)
#define ADD_POWDER_MAX_DISTANCE (270) //超过最大距离后,加料
#define DRAW_TIME (1800000)   //粉料可用时间[秒]
#define ADD_POWDER_TIME (70)  //加料需要的时间[秒]
namespace phoenix {
namespace framework {

MsgServer::MsgServer(Task *manager)
    : Task(TASK_ID_MSG_SERVER, "Message Server", manager) {
  ros_node_ = Nullptr_t;
  running_flag_msg_server_ = false;
  shared_data_ = SharedData::instance();
  servo_control_ = sensor::servo::ServoControl::instance();
  enable_draw_line_ = false;
  enable_survey_ = false;
  enbale_write_word_ = false;
  is_draw_line_started_ = false;
  is_survey_started_ = false;
  is_write_word_started_ = false;
  is_started_add_powder_ = false;
  draw_time_count_ = 0;
  add_powder_count_ = 0;
  init_x_ = 100;
  init_y_ = 300;
  draw_line_x_ = 0;
  draw_line_y_ = 300;
}

void MsgServer::SetParameter(Int32_t argv[]) {
    init_x_ = argv[1];
    init_y_ = argv[2];
    draw_line_x_ = argv[3];
    draw_line_y_ = argv[4];
    Int32_t parameter[8] = {0};
    parameter[0] = argv[0];
    servo_control_->SetParameter(parameter);
}

bool MsgServer::Start() {
  bool ret = true;
  if (Nullptr_t != ros_node_) {
    ret = ros_node_->AddServer("survey_server", &MsgServer::HandleSurveyRequest,
                               this);
    if (false == ret) {
      LOG_ERR << "Failed to add  survey server .";
    }
    ret = ros_node_->AddServer("draw_line_server",
                               &MsgServer::HandleDrawLineRequest, this);
    if (false == ret) {
      LOG_ERR << "Failed to add  draw line server  .";
    }

    ret = ros_node_->AddServer("write_word_server",
                               &MsgServer::HandleWriteWordRequest, this);
    if (false == ret) {
      LOG_ERR << "Failed to add  write word server  .";
    }
  }

#if DEBUG_SERVO
  if (servo_control_->Start()) {          //滑轨开始回原点
    servo_control_->WaitAxisMoveDone(0);  //等待0轴回原点完成
    servo_control_->WaitAxisMoveDone(1);  //等待1轴回原点完成
    LOG_INFO(3) << "Servo  go back to home finished.";

    AlarmLamp::AlarmLampPin lampPin(0, 1, 2);
    alarm_lamp.SetLampParam(AlarmLamp::FlashingMode::FLASHING_MODE_STEADY_ON,
                            AlarmLamp::ALARM_COLOR_GREEN, lampPin);
    alarm_lamp.Start();
    LOG_INFO(3) << "Alarm lamp start.";
  } else {
    LOG_ERR << "Failed to start  servo.";
    return (false);
  }
#endif

  if (running_flag_msg_server_) {
    return (true);
  }
  LOG_INFO(3) << "Create thread of task servo control...";
  running_flag_msg_server_ = true;
  thread_msg_server_ =
      boost::thread(boost::bind(&MsgServer::TheadProcessRequest, this));
  LOG_INFO(3) << "Create thread of task servo control... [OK]";
  std::cout << "Start id:" << boost::this_thread::get_id() << std::endl;

  return (ret);
}

bool MsgServer::Stop() {
  if (running_flag_msg_server_) {
    running_flag_msg_server_ = false;
    LOG_INFO(3) << "Stop thread of task servo control ...";
    bool ret = thread_msg_server_.timed_join(boost::posix_time::seconds(1));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread of task servo control to stop.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop thread of task servo control ... [OK]";
    } else {
      LOG_INFO(3) << "Stop thread of task servo control ... [NG]";
    }
  }
}
//测量请求
bool MsgServer::HandleSurveyRequest(survey::SurveyServer::Request &req,
                                    survey::SurveyServer::Response &res) {
  LOG_INFO(3) << "HandleSurveyRequest start";
  survey_req_ = req;
  enable_survey_ = true;
  Dormancy dormancy(500);
  //等待测量完成
  while (enable_survey_) {
    dormancy.Sleep();
    continue;
  }
  dormancy.Sleep();
  servo_control_->EnableHopperVibrate(false);
  enable_survey_ = false;
  res = survey_res_;
  LOG_INFO(3) << "HandleSurveyRequest finished.";

  return (true);
}

//处理测量
void MsgServer::ProcessSurvey() {
  int count = 0;
  is_survey_started_ = true;
  ad_msg::RtkResult rtk_result;
  ad_msg::LaserResult laser_result;
  ad_msg::Imu imu;
  ad_msg::Gnss gnss;  //主要取航向角
  //当前坐标与测量坐标
  Float64_t utm_X0 = 0.0, utm_Y0 = 0.0, utm_Z0 = 0.0, utm_X = 0.0, utm_Y = 0.0;
  // RTK定位点与导轨原点的标定数据
  //采用HXXT定位的标定数据
  Float64_t a = 0.48, b = -0.76, c = -0.409, d = 0.0;
  //采用ZHD定位的标定数据
  // Float64_t a = 0.4581, b = -0.7823, c = -0.49, d = 0.0;
  Float64_t roll = 0.0, pitch = 0.0, yaw = 0.0;  //车身姿态
  Float64_t H = 0.0, Z = 0.0, L = 0.0, D = 0.0;
  Float32_t m = 0.0, n = 0.0;  //导轨运动行程
  common::LLtoUTM(survey_req_.lat, survey_req_.lon, &utm_Y, &utm_X);
  // LOG_INFO(3) << "待测点经纬度:[" << survey_req_.lat << "," <<
  // survey_req_.lon
  //             << "]";
  // LOG_INFO(3) << "待测点UTM:[" << utm_X << "," << utm_Y << "]";
  std::cout << "******************待测点位置*******************" << std::endl;
  std::cout << "X:" << std::setprecision(20) << survey_req_.lat << std::endl;
  std::cout << "Y:" << std::setprecision(20) << survey_req_.lon << std::endl;
  std::cout << "UTM_X:" << std::setprecision(20) << utm_X << std::endl;
  std::cout << "UTM_Y:" << std::setprecision(20) << utm_Y << std::endl;
  std::cout << "*************************************" << std::endl;
  rtk_result.Clear();
  imu.Clear();
  gnss.Clear();
  shared_data_->GetRtkResult(&rtk_result);
  shared_data_->GetImuData(&imu);
  shared_data_->GetGnssData(&gnss);
  if (!rtk_result.msg_head.valid) {
    LOG_INFO(3) << "使用车辆定位数据计算导轨行程";
    utm_X0 = gnss.x_utm;
    utm_Y0 = gnss.y_utm;
    utm_Z0 = (gnss.altitude + gnss.z_utm);
  } else {
    LOG_INFO(3) << "使用 RTK 定位数据计算导轨行程";
    utm_X0 = rtk_result.utm_result.x;
    utm_Y0 = rtk_result.utm_result.y;
    utm_Z0 = rtk_result.gnss_result.height + rtk_result.utm_result.z;
  }

  roll = imu.roll;
  pitch = imu.pitch;
  yaw = gnss.heading_gnss;

  Float64_t R[3][3] = {
      {cos(pitch) * cos(yaw), cos(pitch) * sin(yaw), -sin(pitch)},
      {sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw),
       sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw),
       sin(roll) * cos(pitch)},
      {cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw),
       cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw),
       cos(roll) * cos(pitch)}};
  m = ((utm_X - utm_X0) * R[1][1] - (utm_Y - utm_Y0) * R[1][0] -
       c * R[2][0] * R[1][1] + c * R[2][1] * R[1][0]) /
          (R[0][0] * R[1][1] - R[1][0] * R[0][1]) -
      a;
  n = ((utm_X - utm_X0) * R[0][1] - (utm_Y - utm_Y0) * R[0][0] -
       c * R[0][1] * R[2][0] + c * R[2][1] * R[0][0]) /
          (R[0][1] * R[1][0] - R[1][1] * R[0][0]) -
      b;

  //等待导轨运动到m,n位置
  // m、n单位是m，运动控制卡运动单位是mm，插补方式运动控制，只需要判断主轴的运动状态
  // LOG_INFO(3) << "RTK经纬度:[" << rtk_result.gnss_result.lat << ","
  //             << rtk_result.gnss_result.lon << "]";
  // LOG_INFO(3) << "RTK UTM:[" << rtk_result.utm_result.x << ","
  //             << rtk_result.utm_result.y << "]";
  // LOG_INFO(3) << "RTK 高程:["
  //             << rtk_result.utm_result.z + rtk_result.gnss_result.height <<
  //             "]";
  std::cout << "******************车当前位置*******************" << std::endl;
  std::cout << "X:" << std::setprecision(20) << gnss.latitude << std::endl;
  std::cout << "Y:" << std::setprecision(20) << gnss.longitude << std::endl;
  std::cout << "Heading:" << std::setprecision(20) << gnss.heading_gnss
            << std::endl;
  std::cout << "utm_X:" << std::setprecision(20) << gnss.x_utm << std::endl;
  std::cout << "utm_Y:" << std::setprecision(20) << gnss.y_utm << std::endl;
  std::cout << "UTM_Heading:" << std::setprecision(20) << gnss.heading_utm
            << std::endl;
  std::cout << "*************************************" << std::endl;
  // LOG_INFO(3) << "车当前经纬度:[" << gnss.latitude << "," << gnss.longitude
  //             << "]";
  // LOG_INFO(3) << "车当前 UTM:[" << gnss.x_utm << "," << gnss.y_utm << "]";
  if (survey_req_.index == "" &&
      survey_req_.lat <= phoenix::common::NumLimits<Float64_t>::epsilon() &&
      survey_req_.lon <= phoenix::common::NumLimits<Float64_t>::epsilon()) {
    std::cout << "################默认测量###############" << std::endl;
    m = init_x_, n = init_y_;
    std::cout << "#######################################" << std::endl;
  }
  LOG_INFO(3) << "车当前 高程:[" << gnss.z_utm + gnss.altitude << "]";
  LOG_INFO(3) << "导轨行程:[" << m << "," << n << "]";
#if DEBUG_SERVO
  // m = 100;
  // n = 300;
  servo_control_->MoveToPosAbs(m * 1000.0, n * 1000.0);
  servo_control_->WaitAxisMoveDone(0);
  servo_control_->WaitAxisMoveDone(1);

  Float32_t current_x = 0.0, current_y = 0.0;
  servo_control_->GetCurrentPos(0, &current_x);
  servo_control_->GetCurrentPos(1, &current_y);
  std::cout << "******************RTK当前位置*******************" << std::endl;
  std::cout << "X:" << std::setprecision(20) << rtk_result.gnss_result.lat
            << std::endl;
  std::cout << "Y:" << std::setprecision(20) << rtk_result.gnss_result.lon
            << std::endl;
  std::cout << "H:" << std::setprecision(20) << rtk_result.gnss_result.height
            << std::endl;
  std::cout << "utm_X:" << std::setprecision(20) << rtk_result.utm_result.x
            << std::endl;
  std::cout << "utm_Y:" << std::setprecision(20) << rtk_result.utm_result.y
            << std::endl;
  std::cout << "utm_Z:" << std::setprecision(20) << rtk_result.utm_result.z
            << std::endl;
  std::cout << "RTK高程:" << std::setprecision(20)
            << rtk_result.utm_result.z + rtk_result.gnss_result.height
            << std::endl;

  std::cout << "*************************************" << std::endl;
#endif
  //获取激光测距数据
  framework::Dormancy dormancy(100);
  laser_result.Clear();
  while (!laser_result.msg_head.valid) {
    shared_data_->GetLaserResult(&laser_result);
    dormancy.Sleep();
  }
  H = laser_result.height;

  servo_control_->EnableHopperVibrate(true);
  dormancy.Sleep();
  shared_data_->GetGnssData(&gnss);
  utm_Z0 = (gnss.altitude + gnss.z_utm);
  // utm_Z0 = gnss.altitude + 13.246;
  Z = (m + a) * R[0][2] + (n + b) * R[1][2] + (D + c) * R[2][2] + utm_Z0 - H -
      d;
  // D = 0, d = 激光设备的长度, H = 激光测量的对地垂直高度

  survey_res_.index = survey_req_.index;
  Float64_t x = 0.0, y = 0.0;
  Int32_t zone = 0;
  if (rtk_result.msg_head.valid) {
    x = (a + m) * R[0][0] + (b + n) * R[1][0] + (c - H) * R[2][0] +
        rtk_result.utm_result.x;
    y = (a + m) * R[0][1] + (b + n) * R[1][1] + (c - H) * R[2][1] +
        rtk_result.utm_result.y;
    zone = GetLongZone(rtk_result.gnss_result.lon);
  } else {
    x = (a + m) * R[0][0] + (b + n) * R[1][0] + (c - H) * R[2][0] + gnss.x_utm;
    y = (a + m) * R[0][1] + (b + n) * R[1][1] + (c - H) * R[2][1] + gnss.y_utm;
    zone = GetLongZone(gnss.longitude);
  }
  //!仅限北半球，南半球需要增加判断
  std::string zone_id = std::to_string(zone) + "N";
  common::UTMtoLL(y, x, zone_id.c_str(), &survey_res_.lat, &survey_res_.lon);
  survey_res_.height = Z + HEIGHT_OFFSET_VALUE;

  LOG_INFO(3) << "测量经纬度:[" << survey_res_.lat << "," << survey_res_.lon
              << "]";
  LOG_INFO(3) << "测量 UTM:[" << x << "," << y << "]";
  LOG_INFO(3) << "激光 高程:[" << H << "]";
  LOG_INFO(3) << "测量 高程:[" << survey_res_.height << "]";
  LOG_INFO(3) << "IMU(roll,pitch,yaw):[" << roll << "," << pitch << "," << yaw
              << "]";
  is_survey_started_ = false;
}

//划线请求
bool MsgServer::HandleDrawLineRequest(survey::DrawLine::Request &req,
                                      survey::DrawLine::Response &res) {
  LOG_INFO(3) << "HandleDrawLineRequest:" << req.status;
  is_draw_line_started_ = true;
  res.index = req.index;
  draw_line_req_ = req;
  enable_draw_line_ = req.status;
#if DEBUG_SERVO
  if (enable_draw_line_) {
      servo_control_->MoveToPosAbs(draw_line_x_, draw_line_y_);
      servo_control_->WaitAxisMoveDone(0);
      servo_control_->WaitAxisMoveDone(1);
      Dormancy dormancy(200);
      dormancy.Sleep();
      servo_control_->EnableHopperVibrate(true);
  } else {  //划线结束
    servo_control_->EnableHopperVibrate(false);
    LOG_INFO(3) << "HandleDrawLineRequest finished.";
  }
#endif
  res.status = req.status;
  is_draw_line_started_ = false;
  return (true);
}

//粉料监测
void MsgServer::PowderMonitor(bool add_powder) {
    if (is_write_word_started_ || is_survey_started_ || is_draw_line_started_) {
        return;
    }

    ad_msg::LaserLiResult laser_li_result;
    shared_data_->GetLaserLiResult(&laser_li_result);
    if (add_powder || ((!is_started_add_powder_) &&
                       (laser_li_result.distance > ADD_POWDER_MAX_DISTANCE))) {
        //开始加料
        add_powder_count_ = 0;
        msg_sender_->SendStringMsg("StopADAS");
        LOG_INFO(3) << "SendStringMsg StopADAS";
        is_started_add_powder_ = true;
        StartAddPowder();
        LOG_INFO(3) << "StartAddPowder.";
    }

    uint32 add_powder_io = 0;
    uint32 powder_full_io = 0;
#if DEBUG_SERVO
    servo_control_->GetOutPort(3, &add_powder_io); //查看放料口状态
    servo_control_->GetInPort(6, &powder_full_io);
#endif
    if (add_powder_io == 1) {
        add_powder_count_++;
        //结束加料
        if (add_powder_count_ >= ADD_POWDER_TIME * 10 ||
            powder_full_io == 1) { //计时50s后完成加料或料加满
            add_powder_count_ = 0;
            if (powder_full_io == 0) {
                alarm_lamp.SetAlarmColor(
                    AlarmLamp::AlarmColor::ALARM_COLOR_YELLOW);
            } else {
                alarm_lamp.SetAlarmColor(
                    AlarmLamp::AlarmColor::ALARM_COLOR_GREEN);
            }
            FinishedAddPowder();
            is_started_add_powder_ = false;
            msg_sender_->SendStringMsg("StartADAS");
            LOG_INFO(3) << "SendStringMsg StartADAS";
            LOG_INFO(3) << "FinishedAddPowder.";
        }
    }

    return;
}

void MsgServer::ManualAddMaterial() {
  if (is_write_word_started_ || is_survey_started_ || is_draw_line_started_) {
    msg_sender_->SendStringMsg("ManualAddMaterialRes");
    return;
  }
  if (is_started_add_powder_) {
      add_powder_count_ = phoenix::common::NumLimits<Uint32_t>::max() - 6000000;
      PowderMonitor(false);
  } else {
      PowderMonitor(true);
  }

  Dormancy dormancy(500);
  dormancy.Sleep();
  msg_sender_->SendStringMsg("ManualAddMaterialRes");
}

bool MsgServer::HandleWriteWordRequest(survey::WriteWord::Request &req,
                                       survey::WriteWord::Response &res) {
  LOG_INFO(3) << "HandleWriteWordRequest:" << req.wrod.c_str();
  std::string word(req.wrod.rbegin(), req.wrod.rend());
  res.index = req.index;
  res.status = false;
  for (size_t i = 0; i < word.size(); i++) {
      std::string char_name;
      char_name.append(1, word[i]);
      std::map<std::string, std::vector<std::string>>::iterator iter =
          svg_word_describe_.find(char_name);
      if (iter != svg_word_describe_.end()) {
          LOG_INFO(3) << "Start Process WriteWord : " << char_name.c_str();
          Dormancy dormancy(500);
          //等待加料完成
          while (is_started_add_powder_) {
              dormancy.Sleep();
              LOG_INFO(3) << "###############add powder###############";
              continue;
          }
          is_write_word_started_ = true;
          ProcessWriteWord(iter->second[0], iter->second[1]);
          is_write_word_started_ = false;
          LOG_INFO(3) << "End Process WriteWord";
          if (i < word.size() - 1) {
              msg_sender_->SendStringMsg("StartADAS");
              LOG_INFO(3) << "SendStringMsg StartADAS";
              ad_msg::Gnss start_gnss;
              ad_msg::Gnss now_gnss;
              SharedData::instance()->GetGnssData(&start_gnss);
              SharedData::instance()->GetGnssData(&now_gnss);
              phoenix::common::GpsPoint point1;
              phoenix::common::GpsPoint point2;
              point1.longitude = start_gnss.longitude;
              point1.latitude = start_gnss.latitude;
              point2.longitude = now_gnss.longitude;
              point2.latitude = now_gnss.latitude;
              while (phoenix::common::CalcSphericalDistance(point1, point2) <
                     WORD_WIDE_VALUE) {
                  SharedData::instance()->GetGnssData(&now_gnss);
                  point2.longitude = now_gnss.longitude;
                  point2.latitude = now_gnss.latitude;
                  std::this_thread::sleep_for(std::chrono::milliseconds(100));
              }
              msg_sender_->SendStringMsg("StopADAS");
              LOG_INFO(3) << "SendStringMsg StopADAS";
          }
      } else {
          LOG_ERR << "Failed to find svg word describe (" << char_name.c_str()
                  << ").";
      }
  }
#if DEBUG_SERVO
  servo_control_->MoveToPosAbs(init_x_, init_y_);
#endif
  res.status = true;
  return (true);
}

void MsgServer::ProcessWriteWord(const std::string &transform,
                                 const std::string &path) {
  framework::SvgPathContext ctx;
  svgpp::value_parser<svgpp::tag::type::path_data>::parse(
      svgpp::tag::attribute::d(), ctx, path, svgpp::tag::source::attribute());

  int32_t table_size = 0;
  //获取运动控制卡的Table Size
  servo_control_->GetTableSize(&table_size);
  std::cout << "Table Size : " << table_size << std::endl;

  phoenix::common::CamPathPrase cam_path;
  std::vector<phoenix::common::Vec2d> sub_path_points;

  const Float32_t x_max = servo_control_->GetMaxCoorX();
  const Float32_t y_max = servo_control_->GetMaxCoorY();
  const Float32_t zoom_ratio = 1;
  //将svg 的所有路径点坐标　存入凸轮当中
  while (ctx.GetNextSubPathPoints(sub_path_points)) {
    if (sub_path_points.size() <= 0) {
      continue;
    }
    framework::SvgTransformContext transform_ctx(&sub_path_points);
    svgpp::value_parser<svgpp::tag::type::transform_list>::parse(
        svgpp::tag::attribute::transform(), transform_ctx, transform,
        svgpp::tag::source::attribute());

    for (Int32_t i = 0; i < sub_path_points.size(); i++) {
        sub_path_points[i].set_x((x_max - sub_path_points[i].x()) / zoom_ratio);
        sub_path_points[i].set_y(sub_path_points[i].y() / zoom_ratio);
        if ((!servo_control_->CheckAxisXCoor(sub_path_points[i].x())) ||
            (!servo_control_->CheckAxisYCoor(sub_path_points[i].y()))) {
            LOG_ERR << "Out of Axis move range.";
        }
      cam_path.AddPathPoint(sub_path_points[i]);
    }
  }
  //将路径点转成凸轮有效数据
  cam_path.PointsToCamPath();
  ad_msg::WriteWordResult write_word_result;
  write_word_result.stroke_path_points.clear();
  framework::Dormancy dormancy(200);
#if DEBUG_SERVO
  for (int32_t i = 0; i < cam_path.GetStrokeSize(); i++) {
      phoenix::common::Vec2d storke_starting_point =
          cam_path.GetStrokeStartingPoint(i);
      std::vector<Float32_t> coordinate_x = cam_path.GetCamCoordinatesX(i);
      std::vector<Float32_t> coordinate_y = cam_path.GetCamCoordinatesY(i);

      if (table_size < coordinate_x.size() + coordinate_y.size()) {
          LOG_ERR
              << "There is not enough Table Size on the motion control card ";
          break;
      }
      int32_t table_start_y = static_cast<int32_t>(table_size / 2);
      //设置控制卡Table
      servo_control_->SetTable(0, coordinate_x.size(), coordinate_x.data());
      servo_control_->SetTable(table_start_y, coordinate_y.size(),
                               coordinate_y.data());
      //开始移动到起点
      servo_control_->MoveToPosAbs(storke_starting_point.x(),
                                   storke_starting_point.y());
      //移动完成
      while (servo_control_->GetAxisIsRunning(0) ||
             servo_control_->GetAxisIsRunning(1)) {

          dormancy.Sleep();
      }
      //打开喷墨
      servo_control_->EnableHopperVibrate(true);
      for (int i = 0; i < 4; i++) {
          dormancy.Sleep();
      }

      float axis_x_units = servo_control_->GetAxisUnits(0);
      float axis_y_units = servo_control_->GetAxisUnits(1);
      //开始移动写字(笔画)
      servo_control_->CamMove(0, 0, coordinate_x.size() - 1, axis_x_units,
                              40 * coordinate_x.size());
      servo_control_->CamMove(1, table_start_y,
                              table_start_y + coordinate_y.size() - 1,
                              axis_y_units, 40 * coordinate_x.size());

      //等待移动完成
      std::vector<phoenix::common::Vec2d> path_points;

      while (servo_control_->GetAxisIsRunning(0) ||
             servo_control_->GetAxisIsRunning(1)) {
          float x = servo_control_->GetAxisMpos(0);
          float y = servo_control_->GetAxisMpos(1);
          path_points.push_back(phoenix::common::Vec2d(x, y));
          write_word_result.stroke_path_points[i] = path_points;
          shared_data_->SetWriteWordResult(write_word_result);
          dormancy.Sleep();
      }
      //关闭喷墨
      servo_control_->EnableHopperVibrate(false);
      framework::Dormancy dormancy_close(650);
      dormancy_close.Sleep(); //等待关闭完成
  }
#else
  {
      //测试写字实时展示
      std::vector<phoenix::common::Vec2d> path_points;
      for (int32_t i = 0; i < cam_path.GetStrokeSize(); i++) {
          std::vector<Float32_t> coordinate_x = cam_path.GetAbsCoordinatesX(i);
          std::vector<Float32_t> coordinate_y = cam_path.GetAbsCoordinatesY(i);
          LOG_INFO(3) << "coordinate_x size : " << coordinate_x.size();
          for (int32_t index = 0; index < coordinate_x.size(); index++) {
              float x = coordinate_x[index];
              float y = coordinate_y[index];
              path_points.push_back(phoenix::common::Vec2d(x, y));
              write_word_result.stroke_path_points[i] = path_points;
              shared_data_->SetWriteWordResult(write_word_result);
              dormancy.Sleep();
          }
          path_points.clear();
      }
  }
#endif
  write_word_result.is_complete = true;
  shared_data_->SetWriteWordResult(write_word_result);
  dormancy.Sleep();
  return;
}

//测量任务处理
void MsgServer::TheadProcessRequest() {
  Dormancy dormancy(100);
  while (running_flag_msg_server_) {
      PowderMonitor(false);
      if (enable_survey_) {
          ProcessSurvey();
          enable_survey_ = false;
      }
    dormancy.Sleep();
  }
}

//开始添加粉末动作
void MsgServer::StartAddPowder() {
  framework::Dormancy dormancy(100);
#if DEBUG_SERVO
  servo_control_->EnableHopperVibrate(false);
  dormancy.Sleep();
  servo_control_->MoveXAxis(servo_control_->GetMinCoorX());
  dormancy.Sleep();
  servo_control_->WaitAxisMoveDone(0);
  servo_control_->MoveYAxis(servo_control_->GetMaxCoorY());
  dormancy.Sleep();
  servo_control_->WaitAxisMoveDone(1);
  servo_control_->EnableRepositoryVibrate(true);
  phoenix::ad_msg::WritingWordInfo writeProcessMsg;
  writeProcessMsg.SetWritingWordData(0, 0, 0, 0, 1);
  msg_sender_->SendWritingProcessInfo(writeProcessMsg);
#endif
}

void MsgServer::FinishedAddPowder() {
#if DEBUG_SERVO
    framework::Dormancy dormancy(100);
    servo_control_->WaitAxisMoveDone(0);
    servo_control_->WaitAxisMoveDone(1);
    //关闭电磁阀和搅拌器
    servo_control_->EnableRepositoryVibrate(false);
    dormancy.Sleep();
    dormancy.Sleep();
    dormancy.Sleep();
    dormancy.Sleep();
    dormancy.Sleep();
    // x轴先运动
    servo_control_->MoveXAxis(init_x_);
    dormancy.Sleep();
    servo_control_->WaitAxisMoveDone(0);
    // y轴运动
    servo_control_->MoveYAxis(init_y_);
    dormancy.Sleep();
    servo_control_->WaitAxisMoveDone(1);
    // servo_control_->MoveXAxis(0);
    // dormancy.Sleep();
    // servo_control_->WaitAxisMoveDone(0);
    phoenix::ad_msg::WritingWordInfo writeProcessMsg;
    writeProcessMsg.SetWritingWordData(100, servo_control_->GetMaxCoorY() - 300,
                                       0, 0, 0);
    msg_sender_->SendWritingProcessInfo(writeProcessMsg);
    if (enable_draw_line_) {
        servo_control_->MoveToPosAbs(draw_line_x_, draw_line_y_);
        servo_control_->EnableHopperVibrate(true);
    }
#endif
}

bool MsgServer::AddSvgWordDescribe(std::string &name, std::string &transform,
                                   std::string &path) {
  std::map<std::string, std::vector<std::string>>::iterator it =
      svg_word_describe_.find(name);
  if (it == svg_word_describe_.end()) {
    svg_word_describe_[name] = {transform, path};
    LOG_INFO(3) << "Success to add char path [" << name.c_str() << "].";
  } else {
    LOG_INFO(3) << "Failed to add char path [" << name.c_str() << "]"
                << "already exist!";
    return (false);
  }
  return (true);
}

int MsgServer::GetLongZone(double longitude) {
  double longZone = 0.0;
  if (longitude < 0.0) {
    longZone = ((180.0 + longitude) / 6.0) + 1;
  } else {
    longZone = (longitude / 6.0) + 31;
  }
  return static_cast<int>(longZone);
}

}  // namespace framework
}  // namespace phoenix