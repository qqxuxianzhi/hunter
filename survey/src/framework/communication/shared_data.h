/******************************************************************************
 ** 共享数据存储
 ******************************************************************************
 *
 *  共享数据存储
 *
 *  @file       shared_data.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_SHARED_DATA_H_
#define PHOENIX_FRAMEWORK_SHARED_DATA_H_

#include "ad_msg.h"
#include "common/module_status.h"
#include "msg_event_reporting.h"
#include "os/mutex.h"
#include "utils/macros.h"

namespace phoenix {
namespace framework {

class SharedData {
 public:
  ~SharedData();

  // Module status
  void SetModuleStatusList(const ad_msg::ModuleStatusList& data) {
    common::os::WriteLockHelper lock(lock_module_status_list_);

    module_status_list_ = data;
  }
  void GetModuleStatusList(ad_msg::ModuleStatusList* data) {
    common::os::ReadLockHelper lock(lock_module_status_list_);

    *data = module_status_list_;
  }

  void SetGnssData(const ad_msg::Gnss& data) {
    common::os::WriteLockHelper lock(lock_gnss_data_);
    gnss_data_ = data;
  }
  void GetGnssData(ad_msg::Gnss* data) {
    common::os::ReadLockHelper lock(lock_gnss_data_);
    *data = gnss_data_;
  }

  void SetImuData(const ad_msg::Imu& data) {
    common::os::WriteLockHelper lock(lock_imu_data_);
    imu_data_ = data;
  }
  void GetImuData(ad_msg::Imu* data) {
    common::os::ReadLockHelper lock(lock_imu_data_);
    *data = imu_data_;
  }

  // Chassis information
  void SetChassis(const ad_msg::Chassis& data) {
    common::os::WriteLockHelper lock(lock_chassis_);

    chassis_ = data;
  }
  void GetChassis(ad_msg::Chassis* data) {
    common::os::ReadLockHelper lock(lock_chassis_);

    *data = chassis_;
  }

  void SetLaserResult(const ad_msg::LaserResult& data) {
    common::os::WriteLockHelper lock(lock_laser_result_);
    laser_result_ = data;
  }

  void GetLaserResult(ad_msg::LaserResult* data) {
    common::os::ReadLockHelper lock(lock_laser_result_);
    *data = laser_result_;
  }

  void SetLaserLiResult(const ad_msg::LaserLiResult& data) {
    common::os::WriteLockHelper lock(lock_laser_li_result_);
    laser_li_result_ = data;
  }

  void GetLaserLiResult(ad_msg::LaserLiResult* data) {
    common::os::ReadLockHelper lock(lock_laser_li_result_);
    *data = laser_li_result_;
  }

  void SetRtkResult(const ad_msg::RtkResult& data) {
    common::os::WriteLockHelper lock(lock_rtk_result_);
    rtk_result_ = data;
  }

  void GetRtkResult(ad_msg::RtkResult* data) {
    common::os::ReadLockHelper lock(lock_rtk_result_);
    *data = rtk_result_;
  }

 void SetWriteWordResult(const ad_msg::WriteWordResult& data) {
    common::os::WriteLockHelper lock(lock_write_word_result_);
    write_word_result_ = data;
  }

  void GetWriteWordResult(ad_msg::WriteWordResult* data) {
    common::os::ReadLockHelper lock(lock_write_word_result_);
    *data = write_word_result_;
  }

 private:
  // Module status
  ad_msg::ModuleStatusList module_status_list_;
  common::os::ReadWriteMutex lock_module_status_list_;

  // GNSS
  ad_msg::Gnss gnss_data_;
  common::os::ReadWriteMutex lock_gnss_data_;
  // IMU
  ad_msg::Imu imu_data_;
  common::os::ReadWriteMutex lock_imu_data_;
  // Chassis information
  ad_msg::Chassis chassis_;
  common::os::ReadWriteMutex lock_chassis_;
  // Laser
  ad_msg::LaserResult laser_result_;
  common::os::ReadWriteMutex lock_laser_result_;

  ad_msg::LaserLiResult laser_li_result_;
  common::os::ReadWriteMutex lock_laser_li_result_;

  // Rtk
  ad_msg::RtkResult rtk_result_;
  common::os::ReadWriteMutex lock_rtk_result_;

  //WriteWord
  ad_msg::WriteWordResult write_word_result_;
  common::os::ReadWriteMutex lock_write_word_result_;

 private:
  DECLARE_SINGLETON(SharedData);
};

}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_SHARED_DATA_H_
