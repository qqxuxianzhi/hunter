#ifndef PHOENIX_AD_MSG_MSG_MQTT_H_
#define PHOENIX_AD_MSG_MSG_MQTT_H_

#include <string>

#include "msg_common.h"
#include "utils/com_utils.h"
#include "utils/macros.h"
namespace phoenix {
namespace ad_msg {
enum {
  // 喷粉关闭
  DUSTING_SWITCH_CLOSE =0,
 //喷粉开启
  DUSTING_SWITCH_ON =1,
};

enum {
  // 未加料
  NO_MATERIAL_ADDED =0,
 //加料中
  ADDING_MATERIALS =1,
};

enum {
  // 不移动
  NO_MOBILE =0,
 //正在移动
  MOVING =1,
};

struct WritingWordInfo {
  Float64_t offset_x;
  Float64_t offset_y;
  Int8_t dusting_switch_type;
  Int8_t is_move;
  Int8_t add_material_status;
  void Clear() {
      offset_x = 0.0;
      offset_y = 0.0;
      is_move =NO_MOBILE;
      dusting_switch_type = DUSTING_SWITCH_CLOSE;
      add_material_status=NO_MATERIAL_ADDED;
  }
  void SetWritingWordData(Float64_t x,Float64_t y,
        Int8_t dusting,Int8_t move,Int8_t material){
    offset_x = x;
    offset_y = y;
    dusting_switch_type = dusting;
    is_move = move;
    add_material_status= material;
  }
};

}//namespace ad_msg 
}//namespace phoenix

#endif  // PHOENIX_AD_MSG_MSG_MQTT_H_