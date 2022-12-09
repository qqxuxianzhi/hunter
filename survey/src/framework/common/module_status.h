/******************************************************************************
 ** 模块状态类型定义
 ******************************************************************************
 *
 *  定义各种模块状态
 *
 *  @file       mudule_status.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_MODULE_STATUS_H_
#define PHOENIX_FRAMEWORK_MODULE_STATUS_H_

#include "container/static_vector.h"
#include "utils/com_utils.h"
#include "utils/macros.h"

namespace phoenix {
namespace framework {

enum InternalModuleId {
  INTERNAL_MODULE_ID_INVALID = 0,

  /// Module status
  INTERNAL_MODULE_ID_MSG_RECV_GNSS,
  INTERNAL_MODULE_ID_MSG_RECV_IMU,
  INTERNAL_MODULE_ID_MSG_RECV_CHASSIS,
  INTERNAL_MODULE_ID_MSG_RECV_LASER,
  INTERNAL_MODULE_ID_MSG_RECV_RTK,
  INTERNAL_MAX_MODULE_NUM
};

enum {
  INTERNAL_MODULE_STATUS_OK = 0,
  INTERNAL_MODULE_STATUS_ERR,
  INTERNAL_MODULE_STATUS_ERR_POS_FILTER_FAULT,
  INTERNAL_MODULE_STATUS_ERR_DRIVING_MAP_FAULT,
  INTERNAL_MODULE_STATUS_ERR_OBJ_FILTER_FAULT,
  INTERNAL_MODULE_STATUS_ERR_ACTION_PLANNING_FAULT,
  INTERNAL_MODULE_STATUS_ERR_TRAJECTORY_PLANNING_FAULT,
  INTERNAL_MODULE_STATUS_ERR_VELOCITY_PLANNING_FAULT
};

}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_MODULE_STATUS_H_
