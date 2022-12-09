/******************************************************************************
 ** 任务管理模块
 ******************************************************************************
 *
 *  管理所有的任务(启动、停止、状态监测等)
 *
 *  @file       task_manager.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "task_manager.h"

#include <stdlib.h>
#include <unistd.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "boost/date_time.hpp"
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/xml_parser.hpp"
#if (ENABLE_ROS_NODE)
#include "communication/ros_node.h"
#endif
#if (ENABLE_LCM_NODE)
#include "communication/lcm_node.h"
#endif
#if (ENABLE_UDP_NODE)
#include "communication/udp_node.h"
#endif
#include "communication/msg_receiver.h"
#include "communication/msg_sender.h"
#include "communication/msg_server.h"
#include "communication/shared_data.h"
#include "utils/com_utils.h"
#include "utils/gps_tools.h"
#include "utils/log.h"

#if (DEV_IMU_TYPE == DEV_IMU_TYPE_MPSK)
#include "sensor/imu/mpsk/task_recv_gnss_data_mpsk.h"
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_BDSTAR)
#include "sensor/imu/bdstar/task_recv_gnss_data_bdstar.h"
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_INTERTIAL_LAB)
#include "sensor/imu/intertial_lab/task_recv_gnss_intertial_lab.h"
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_F9K)
#include "sensor/imu/f9k/task_recv_nmea_data.h"
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_ZHD_RTK)
#include "sensor/imu/zhd/task_recv_rtk_data.h"
#else
// 未定义IMU设备
#endif
//姿态传感器
#include "sensor/imu/wit_motion/task_recv_imu_data.h"
//激光测距传感器
#if (DEV_LASER_TYPE == DEV_LASER_TYPE_RANGRING)
#include "sensor/laser_distance/laser_rangring/task_recv_laser_data.h"
#elif (DEV_LASER_TYPE == DEV_LASER_TYPE_JRT)
#include "sensor/laser_distance/jrt/task_recv_jrt_laser_data.h"
#else
//未定义Laser设备
#endif

//加粉工件激光测距传感器
#if (DEV_POWDER_LASER_TYPE == DEV_POWDER_LASER_TYPE_LI)
#include "sensor/laser_distance/li_dar/task_recv_li_laser_data.h"
#else
//未定义Laser设备
#endif

// #define SVG_FILE_SUFFIX "_width_200mm.svg"
#define SVG_FILE_SUFFIX "_width_200mm_h.svg"

namespace phoenix {
namespace framework {

/******************************************************************************/
/** 构造函数
 ******************************************************************************
 *  @param      argc            (in)           输入参数
 *              argv            (in)           输入参数
 *              work_space      (in)           工作空间目录
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       构造函数
 *
 *  <Attention>
 *       None
 *
 */
TaskManager::TaskManager(int argc, char** argv, const std::string& work_space)
    : Task(TASK_ID_MANAGER, "Task Manager"), work_space_(work_space) {
  //, log_file_status_("status") {
  thread_running_flag_check_tasks_status_ = false;

#if (ENABLE_ROS_NODE)
  ros_node_.reset(new RosNode(argc, argv, "survey"));
#endif
#if (ENABLE_LCM_NODE)
  lcm_node_.reset(new LcmNode("udpm://239.255.76.67:7667?ttl=1"));
#endif
#if (ENABLE_UDP_NODE)
  udp_node_.reset(new UdpNode());
#endif

  msg_receiver_.reset(new MsgReceiver(this));
  msg_sender_.reset(new MsgSender(this));
  msg_server_.reset(new MsgServer(this));
  Int32_t communication_mode = 0;
  if (0 == communication_mode) {
#if (ENABLE_ROS_NODE)
    msg_receiver_->SetRosNode(ros_node_.get());
    msg_sender_->SetRosNode(ros_node_.get());
    msg_server_->SetRosNode(ros_node_.get());
#endif
#if (ENABLE_LCM_NODE)
    msg_receiver_->SetLcmNode(Nullptr_t);
    msg_sender_->SetLcmNode(Nullptr_t);
#endif
#if (ENABLE_UDP_NODE)
    msg_receiver_->SetUdpNode(Nullptr_t);
    msg_sender_->SetUdpNode(Nullptr_t);
#endif
  } else if (1 == communication_mode) {
#if (ENABLE_ROS_NODE)
    msg_receiver_->SetRosNode(Nullptr_t);
    msg_sender_->SetRosNode(Nullptr_t);
    msg_server_->SetRosNode(Nullptr_t);

#endif
#if (ENABLE_LCM_NODE)
    msg_receiver_->SetLcmNode(lcm_node_.get());
    msg_sender_->SetLcmNode(lcm_node_.get());
#endif
#if (ENABLE_UDP_NODE)
    msg_receiver_->SetUdpNode(Nullptr_t);
    msg_sender_->SetUdpNode(Nullptr_t);
#endif
  } else if (2 == communication_mode) {
#if (ENABLE_ROS_NODE)
    msg_receiver_->SetRosNode(Nullptr_t);
    msg_sender_->SetRosNode(Nullptr_t);
    msg_server_->SetRosNode(Nullptr_t);
#endif
#if (ENABLE_LCM_NODE)
    msg_receiver_->SetLcmNode(Nullptr_t);
    msg_sender_->SetLcmNode(Nullptr_t);
#endif
#if (ENABLE_UDP_NODE)
    msg_receiver_->SetUdpNode(udp_node_.get());
    msg_sender_->SetUdpNode(udp_node_.get());
#endif
  } else {
#if (ENABLE_ROS_NODE)
    msg_receiver_->SetRosNode(Nullptr_t);
    msg_sender_->SetRosNode(Nullptr_t);
#endif
#if (ENABLE_LCM_NODE)
    msg_receiver_->SetLcmNode(Nullptr_t);
    msg_sender_->SetLcmNode(Nullptr_t);
#endif
#if (ENABLE_UDP_NODE)
    msg_receiver_->SetUdpNode(Nullptr_t);
    msg_sender_->SetUdpNode(Nullptr_t);
#endif
    LOG_ERR << "Invalid communication mode.";
  }

  msg_server_->SetMsgSender(msg_sender_.get());

#if (DEV_IMU_TYPE == DEV_IMU_TYPE_MPSK)
  task_recv_gnss_data_.reset(new sensor::imu::mpsk::TaskRecvGnssDataMpsk(this));
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_BDSTAR)
  task_recv_gnss_data_.reset(
      new sensor::imu::bdstar::TaskRecvGnssDataBdstar(this));
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_INTERTIAL_LAB)
  task_recv_gnss_data_.reset(
      new sensor::imu::intertial_lab::TaskRecvGnssIntertialLab(this));
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_F9K)
  task_recv_gnss_data_.reset(new sensor::imu::f9k::TaskRecvNmeaData(this));
  task_recv_imu_data_.reset(new sensor::imu::wit_motion::TaskRecvImuData(this));
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_ZHD_RTK)
  task_recv_gnss_data_.reset(new sensor::rtk::TaskRecvRtkData(this));
#else
  // 未定义IMU设备
#endif

#if (DEV_LASER_TYPE == DEV_LASER_TYPE_RANGRING)
  task_recv_laser_data_.reset(new sensor::laser::TaskRecvLaserData(this));
#elif (DEV_LASER_TYPE == DEV_LASER_TYPE_JRT)
  task_recv_laser_data_.reset(new sensor::laser::TaskRecvJRTLaserData(this));
#else
  // 未定义Laser设备
#endif

#if (DEV_POWDER_LASER_TYPE == DEV_POWDER_LASER_TYPE_LI)
  task_powder_li_laser_data_.reset(
      new sensor::laser::TaskRecvLiLaserData(this));
#else
//未定义Laser设备
#endif
}

/******************************************************************************/
/** 析构函数
 ******************************************************************************
 *  @param      none
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       析构函数
 *
 *  <Attention>
 *       None
 *
 */
TaskManager::~TaskManager() { Stop(); }

/******************************************************************************/
/** 启动各个模块
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       启动各个模块
 *
 *  <Attention>
 *       None
 *
 */
Int32_t TaskManager::Start() {
  bool ret = false;

  ReadSvg();
  ReadLocalConfigFromFile();

  SharedData* shared_data = SharedData::instance();

#if (ENABLE_ROS_NODE)
  // Start ROS Node
  ret = ros_node_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start ROS node.";
    return (START_ERR_FAILED_TO_START_ROS);
  }
#endif

#if (ENABLE_LCM_NODE)
  // Start LCM Node
  ret = lcm_node_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start LCM node.";
    return (START_ERR_FAILED_TO_START_LCM);
  }
#endif

#if (ENABLE_UDP_NODE)
  // Start UDP Node
  UdpNode::UdpParam udp_param;
  udp_param.enable_recv = true;
  udp_param.enable_send = true;
  udp_param.rcv_port = 9500;
  udp_param.snd_port = 9500;
  Int32_t mode = 1;
  if (0 == mode) {
    udp_param.mode = UdpNode::UdpParam::MODE_UNICAST;
    strncpy(udp_param.snd_addr, "192.168.1.100",
            sizeof(udp_param.snd_addr) - 1);
  } else if (1 == mode) {
    udp_param.mode = UdpNode::UdpParam::MODE_GROUP;
    strncpy(udp_param.snd_addr, "224.10.10.2", sizeof(udp_param.snd_addr) - 1);
  } else {
    udp_param.mode = UdpNode::UdpParam::MODE_BROADCAST;
    strncpy(udp_param.snd_addr, "192.168.10.102",
            sizeof(udp_param.snd_addr) - 1);
  }
  ret = udp_node_->Start(udp_param);
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start UDP node.";
    return (START_ERR_FAILED_TO_START_UDP);
  }
#endif

  // Start Message receive
  ret = msg_receiver_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start message receiver.";
    return (START_ERR_FAILED_TO_START_MSG_RECV);
  }

  // Start Message send
  ret = msg_sender_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start message sender.";
    return (START_ERR_FAILED_TO_START_MSG_SEND);
  }

  // Start Message Server
  ret = msg_server_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start message server.";
    return (START_ERR_FAILED_TO_START_MSG_SERVER);
  }

#if (!ENTER_PLAYBACK_MODE)

#if (DEV_IMU_TYPE != DEV_IMU_TYPE_UNDEFINED)
  ret = task_recv_gnss_data_->Start();
  if (false == ret) {
    // Stop();
    LOG_ERR << "Failed to start receiving gnss data.";
    // return (START_ERR_FAILED_TO_START_GNSS);
  }

  ret = task_recv_laser_data_->Start();
  if (false == ret) {
    LOG_ERR << "Failed to start receiving Laser data.";
  }

  ret = task_powder_li_laser_data_->Start();
  if (false == ret) {
      LOG_ERR << "Failed to start receiving powder Laser data.";
  }

  // ret = task_recv_imu_data_->Start();
  // if (false == ret) {
  //   LOG_ERR << "Failed to start receiving Imu data.";
  // }
#endif

#endif  // #if (!ENTER_PLAYBACK_MODE)

#if (ENABLE_UDP_NODE)
  ret = udp_node_->StartReceiving();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start UDP receiving.";
    return (START_ERR_FAILED_TO_START_UDP);
  }
#endif

  // Start Thread(Check tasks status)
  if (true != thread_running_flag_check_tasks_status_) {
    LOG_INFO(3) << "Start Check Tasks Status Thread ...";
    thread_running_flag_check_tasks_status_ = true;
    thread_check_tasks_status_ =
        boost::thread(boost::bind(&TaskManager::ThreadCheckTasksStatus, this));
    LOG_INFO(3) << "Start Check Tasks Status Thread ... [OK]";
  }

  return (START_OK);
}

/******************************************************************************/
/** 停止各个模块
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       停止各个模块
 *
 *  <Attention>
 *       None
 *
 */
bool TaskManager::Stop() {
  // Stop Thread(Check tasks status)
  if (true == thread_running_flag_check_tasks_status_) {
    LOG_INFO(3) << "Stop Check Tasks Status Thread ...";
    thread_running_flag_check_tasks_status_ = false;
    bool ret =
        thread_check_tasks_status_.timed_join(boost::posix_time::seconds(2));
    if (false == ret) {
      LOG_ERR << "Failed to wait for thread \"Check tasks status\" exit.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop Check Tasks Status Thread... [OK]";
    } else {
      LOG_INFO(3) << "Stop Check Tasks Status Thread... [NG]";
    }
  }

#if (!ENTER_PLAYBACK_MODE)

#if (DEV_IMU_TYPE != DEV_IMU_TYPE_UNDEFINED)
  task_recv_gnss_data_->Stop();
  // task_recv_imu_data_->Stop();
#endif
  task_recv_laser_data_->Stop();
#endif  // #if (!ENTER_PLAYBACK_MODE)

#if (ENABLE_UDP_NODE)
  // Stop UDP Node
  udp_node_->Stop();
#endif
#if (ENABLE_LCM_NODE)
  // Stop LCM Node
  lcm_node_->Stop();
#endif
#if (ENABLE_ROS_NODE)
  // Stop ROS Node
  ros_node_->Stop();
#endif
  return (true);
}

/******************************************************************************/
/** 监测各个模块的状态
 ******************************************************************************
 *  @param      none
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       监测各个模块的状态
 *
 *  <Attention>
 *       None
 *
 */
void TaskManager::ThreadCheckTasksStatus() {
  LOG_INFO(3) << "Check Tasks Status Thread ... [Started]";

  Dormancy dormancy(50);
  for (Int32_t i = 0; i < 10; ++i) {
    dormancy.Sleep();
  }

  while (true == thread_running_flag_check_tasks_status_) {
    work_monitor_.DoWork();

    bool survey_stauts = msg_server_->GetSurveyStatus();
    // LOG_INFO(3) << "survey_stauts:" << survey_stauts;
    Uint8_t laser_status = task_recv_laser_data_->GetLaserStatus();

    if (survey_stauts) {
      // if (laser_status == sensor::laser::TaskRecvJRTLaserData::LASER_OFF &&
      //     laser_status != sensor::laser::TaskRecvJRTLaserData::LASER_OPENING)
      //     {
      //   LOG_INFO(3) << "OpenLaser";
      //   task_recv_laser_data_->OpenLaser();
      // }
      if (laser_status != sensor::laser::TaskRecvJRTLaserData::LASER_AUTO_CNT &&
          laser_status !=
              sensor::laser::TaskRecvJRTLaserData::LASER_AUTO_CNTING) {
        LOG_INFO(3) << "CntinusAuto";
        task_recv_laser_data_->CntinusAuto();
      }
    } else {
      if (laser_status == sensor::laser::TaskRecvJRTLaserData::LASER_AUTO_CNT &&
          laser_status != sensor::laser::TaskRecvJRTLaserData::LASER_EXIT_CNT) {
        LOG_INFO(3) << "CntinusExit";
        task_recv_laser_data_->CntinusExit();
      }
      // if (laser_status == sensor::laser::TaskRecvJRTLaserData::LASER_EXIT_CNT
      // &&
      //     laser_status != sensor::laser::TaskRecvJRTLaserData::LASER_CLOSING)
      //     {
      //   LOG_INFO(3) << "CloseLaser";
      //   task_recv_laser_data_->CloseLaser();
      //}
    }

    // static bool last_status = false;
    // bool status = msg_server_->GetAddPowderStatus();
    // if (last_status != status && status) {
    //   msg_sender_->SendStringMsg("StopADAS");
    //   LOG_INFO(3) << "SendStringMsg StopADAS";
    // }
    // if (last_status != status && !status) {
    //   msg_sender_->SendStringMsg("StartADAS");
    //   LOG_INFO(3) << "SendStringMsg StartADAS";
    // }
    // last_status = status;
    dormancy.Sleep();
  }
  task_recv_laser_data_->CntinusExit();
  task_recv_laser_data_->CloseLaser();
  LOG_INFO(3) << "Check Tasks Status Thread ... [Stopped]";
}

/******************************************************************************/
/** 处理模块消息
 ******************************************************************************
 *  @param      msg             (in)           消息
 *              sender          (in)           发送者
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       处理模块消息
 *
 *  <Attention>
 *       None
 *
 */
bool TaskManager::HandleMessage(const Message& msg, Task* sender) {
  // Must be reentrant function
  SharedData* shared_data = SharedData::instance();

  bool ret = true;
  switch (msg.id()) {
    case MSG_ID_RECV_GNSS_DATA: {
      const MessageRecvGnssData& message =
          dynamic_cast<const MessageRecvGnssData&>(msg);
      shared_data->SetGnssData(*message.gnss());
      // msg_sender_->SendGnssData(*message.gnss());

      // Monitor
      work_monitor_.FeedDog_RecvGnss();
    } break;

    case MSG_ID_RECV_IMU_DATA: {
      const MessageRecvImuData& message =
          dynamic_cast<const MessageRecvImuData&>(msg);
      shared_data->SetImuData(*message.imu());
      // msg_sender_->SendImuData(*message.imu());

      // Monitor
      work_monitor_.FeedDog_RecvImu();
    } break;

    case (MSG_ID_CHASSIS): {
      const MessageChassis& message = dynamic_cast<const MessageChassis&>(msg);
      shared_data->SetChassis(*message.chassis());

      // Monitor
      work_monitor_.FeedDog_RecvChassis();
    } break;

    case (MSG_ID_LASER_DATA): {
      const MessageLaserData& message =
          dynamic_cast<const MessageLaserData&>(msg);
      msg_sender_->SendLaserResult(*message.laserResult());
    } break;

    case (MSG_ID_RECV_LASER_DATA): {
      const MessageRecvLaserData& message =
          dynamic_cast<const MessageRecvLaserData&>(msg);
      shared_data->SetLaserResult(*message.laserResult());
      // Monitor
      work_monitor_.FeedDog_RecvLaser();
    } break;
    case (MSG_ID_RECV_LI_LASER_DATA): {
        const MessageRecvLiLaserData &message =
            dynamic_cast<const MessageRecvLiLaserData &>(msg);
        shared_data->SetLaserLiResult(*message.laserLiResult());
        //std::cout << *message.laserLiResult() << std::endl;
    } break;

    case (MSG_ID_RTK_DATA): {
      const MessageRtkData& message = dynamic_cast<const MessageRtkData&>(msg);
      msg_sender_->SendRtkResult(*message.rtkResult());
    } break;

    case (MSG_ID_RECV_RTK_DATA): {
      const MessageRecvRtkData& message =
          dynamic_cast<const MessageRecvRtkData&>(msg);
      shared_data->SetRtkResult(*message.rtkResult());
      // Monitor
      work_monitor_.FeedDog_RecvRtk();
    } break;

    case (MSG_ID_RTK_DATA_SOURCE): {
      const MessageRtkDataSource& message =
          dynamic_cast<const MessageRtkDataSource&>(msg);

      msg_sender_->SendRtkDataSource(*message.rtk_data_source());
    } break;

    case (MSG_ID_PLANNING_STRING): {
      const MessagePlanningString& message =
          dynamic_cast<const MessagePlanningString&>(msg);
      if (*message.planning_string() == "ManualAddMaterialReq") {
        msg_server_->ManualAddMaterial();
      }
    } break;

    default:
      ret = false;
      break;
  }
  return (ret);
}

bool TaskManager::ReadSvg() {
  std::string file_dir = work_space_ + "/conf/";
  std::vector<std::string> char_name = {
      "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D", "E",
      "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T",
      "U", "V", "W", "X", "Y", "Z","a","b","c","d","e","f","g","h","i","j","k",
      "l","m","n","o","p","q","r","s","t","u","v","w","x","y","z","^","~","<",
      "=",">","-","!","(",")","[","]","{","}","@","$","*","\\","&","%","+","#", "."};
  try {
    for (Int32_t i = 0; i < char_name.size(); i++) {
      std::string file_name = file_dir + char_name[i] + SVG_FILE_SUFFIX;
      // std::cout << "\n###### Read svg from file: (Begin) ######\n"
      //           << "  file_name= \"" << file_name << "\"." << std::endl;
      // Create empty property tree object
      boost::property_tree::ptree pt;
      // Load XML file and put its contects in property tree
      boost::property_tree::read_xml(file_name, pt);

      std::string transform = "";
      transform += pt.get<std::string>("svg.g.<xmlattr>.transform", "");
      transform += pt.get<std::string>("svg.g.g.<xmlattr>.transform", "");
      transform += pt.get<std::string>("svg.g.g.path.<xmlattr>.transform", "");

      std::string path = pt.get<std::string>("svg.g.path.<xmlattr>.d", "");
      if (path == "") {
        path = pt.get<std::string>("svg.g.g.path.<xmlattr>.d", "");
      }
      if(path ==""|| transform==""){
        LOG_ERR << "Failed to parsing svg from file  ."<< file_name.c_str() ;
        std::cout << "transform : " << transform << std::endl;
        std::cout << "path : "<< path << std::endl;
      }
 
      msg_server_->AddSvgWordDescribe(char_name[i], transform, path);

      // std::cout << "###### Read svg from file. (End) ######\n" << std::endl;
    }
  } catch (std::exception& e) {
    LOG_ERR << "Failed to read svg from file (" << e.what() << ").";
    return false;
  }
  return true;
}

bool TaskManager::ReadLocalConfigFromFile() {
    std::string config_file = work_space_ + "/conf/local_config.xml";
    LOG_INFO(3) << "\n###### Read configuration from file: (Begin) ######\n"
                << "  file_name= \"" << config_file.c_str() << "\".";
    try {
        boost::property_tree::ptree pt;
        boost::property_tree::read_xml(config_file, pt);
        Int32_t parameter[32] = {0};
        parameter[0] = pt.get<std::int32_t>("config.pulse");
        parameter[1] = pt.get<std::int32_t>("config.init_x");
        parameter[2] = pt.get<std::int32_t>("config.init_y");
        parameter[3] = pt.get<std::int32_t>("config.draw_line_x");
        parameter[4] = pt.get<std::int32_t>("config.draw_line_y");
        msg_server_->SetParameter(parameter);
    } catch (std::exception &e) {
        LOG_ERR << "Failed to read config from file (" << e.what() << ").";
        return false;
    }
    return true;  
}

}  // namespace framework
}  // namespace phoenix
