#ifndef PHOENIX_FRAMEWORK_MSG_SERVER_H_
#define PHOENIX_FRAMEWORK_MSG_SERVER_H_
#include <boost/asio.hpp>
#include <boost/atomic.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

#include "common/message.h"
#include "common/task.h"
#include "communication/ros_node.h"
#include "communication/shared_data.h"
#include "sensor/servo/servo_control.h"
#include "survey/DrawLine.h"
#include "survey/SurveyServer.h"
#include "survey/WriteWord.h"
#include "sensor/servo/alarm_lamp.h"
namespace phoenix {
namespace framework {

class MsgSender;

class MsgServer : public Task {
 public:
  MsgServer(Task *manager);
  bool Start();
  bool Stop();
  void SetRosNode(RosNode *node) { ros_node_ = node; }
  void SetMsgSender(MsgSender *msg_sender) { msg_sender_ = msg_sender; }

  //测量请求
  bool HandleSurveyRequest(survey::SurveyServer::Request &req,
                           survey::SurveyServer::Response &res);
  bool HandleDrawLineRequest(survey::DrawLine::Request &req,
                             survey::DrawLine::Response &res);
  bool HandleWriteWordRequest(survey::WriteWord::Request &req,
                              survey::WriteWord::Response &res);
  bool GetAddPowderStatus() { return (is_started_add_powder_); }
  bool GetSurveyStatus() { return (enable_survey_); }

  bool AddSvgWordDescribe(std::string &name, std::string &transform,
                          std::string &path);
  void ManualAddMaterial();

  void SetParameter(Int32_t argv[]);
  /// @brief
private:
  //处理测量请求
  void ProcessSurvey();
  //粉料监测
  void PowderMonitor(bool add_powder);
  //处理写字请求
  void ProcessWriteWord(const std::string &transform, const std::string &path);
  //处理服务请求
  void TheadProcessRequest();
  //开始添加粉末
  void StartAddPowder();
  //完成添加粉末
  void FinishedAddPowder();
  //根据请求坐标和RTK当前位置，计算导轨需要运动的距离
  void CalcServoMoveDist(const Float64_t &rtk_lat, const Float64_t &rtk_lon,
                         Float32_t *offset_x, Float32_t *offset_y);
  int GetLongZone(double longitude);

 private:
  RosNode *ros_node_;
  MsgSender *msg_sender_;

  boost::atomic_bool running_flag_msg_server_;  //服务线程启动
  boost::thread thread_msg_server_;
  survey::SurveyServer::Request survey_req_;
  survey::DrawLine::Request draw_line_req_;
  survey::SurveyServer::Response survey_res_;
  SharedData *shared_data_;
  sensor::servo::ServoControl *servo_control_;
  AlarmLamp alarm_lamp;
  bool enable_survey_;          //使能测量功能
  bool enable_draw_line_;       //使能划线功能
  bool enbale_write_word_;      //使能写字功能
  bool is_survey_started_;      //开始测量
  bool is_draw_line_started_;   //划线功能是否开始
  bool is_write_word_started_;  //开始写字
  bool is_started_add_powder_;  //是否开始了加料
  bool started_add_powder_;     //开始加料
  Uint32_t draw_time_count_;    //漏料工作计时器
  Uint32_t add_powder_count_;   //加料计时器
  std::map<std::string, std::vector<std::string>>
      svg_word_describe_;  //字符路径描述
  Int32_t init_x_;         // X轴初始位置
  Int32_t init_y_;         // Y轴初始位置
  Int32_t draw_line_x_;    //划线X位置
  Int32_t draw_line_y_;    //划线Y位置
};
}  // namespace framework
}  // namespace phoenix
#endif