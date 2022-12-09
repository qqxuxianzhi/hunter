#ifndef PHOENIX_AD_MSG_MSG_WRITE_WORD_RESULT_H_
#define PHOENIX_AD_MSG_MSG_WRITE_WORD_RESULT_H_

#include <string>
#include <map>
#include <vector>
#include "geometry/vec2d.h"
#include "msg_common.h"
#include "utils/com_utils.h"
#include "utils/macros.h"
namespace phoenix {
namespace ad_msg {

struct WriteWordResult{
   std::map<int32_t, std::vector<phoenix::common::Vec2d>> stroke_path_points;
   bool is_complete;
   WriteWordResult(){
      clear();
   }
   void clear(){
      is_complete = false;
      stroke_path_points.clear();
   }
   bool isEmpty(){
      return (stroke_path_points.size()<=0);
   }
};
}  // namespace ad_msg
}  // namespace phoenix
#endif