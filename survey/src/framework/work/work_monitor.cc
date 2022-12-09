//
#include "work/work_monitor.h"

#include "communication/shared_data.h"
#include "utils/macros.h"

namespace phoenix {
namespace framework {

WorkMonitor::WorkMonitor() { Initialize(); }

WorkMonitor::~WorkMonitor() {}

void WorkMonitor::Initialize() {
  monitor_table_.recv_msg_gnss.Initialize(INTERNAL_MODULE_ID_MSG_RECV_GNSS, 4);
  monitor_table_.recv_msg_imu.Initialize(INTERNAL_MODULE_ID_MSG_RECV_IMU, 4);
  monitor_table_.recv_msg_chassis.Initialize(
      INTERNAL_MODULE_ID_MSG_RECV_CHASSIS, 4);

  monitor_table_.recv_msg_laser.Initialize(INTERNAL_MODULE_ID_MSG_RECV_LASER,
                                           10);
  monitor_table_.recv_msg_rtk.Initialize(INTERNAL_MODULE_ID_MSG_RECV_RTK,
                                         4 * 20);
}

void WorkMonitor::FeedDog_RecvGnss() {
  monitor_table_.recv_msg_gnss.FeedDog(0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvImu() {
  monitor_table_.recv_msg_imu.FeedDog(0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvChassis() {
  monitor_table_.recv_msg_chassis.FeedDog(0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvLaser() {
  monitor_table_.recv_msg_laser.FeedDog(0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvRtk() {
  monitor_table_.recv_msg_rtk.FeedDog(0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

Int32_t WorkMonitor::DoWork() {
  // Check tasks status
  SharedData* shared_data = SharedData::instance();

  ad_msg::ModuleStatus module_status;

  internal_module_status_list_.Clear();

  // message receive (gnss)
  monitor_table_.recv_msg_gnss.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (imu)
  monitor_table_.recv_msg_imu.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (chassis)
  monitor_table_.recv_msg_chassis.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);
  // Laser
  monitor_table_.recv_msg_laser.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);
  // Rtk
  monitor_table_.recv_msg_rtk.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // save
  shared_data->SetModuleStatusList(internal_module_status_list_);

  return (0);
}

}  // namespace framework
}  // namespace phoenix
