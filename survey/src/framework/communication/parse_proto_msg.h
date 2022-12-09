//
#ifndef PHOENIX_FRAMEWORK_PARSE_PROTO_MSG_H_
#define PHOENIX_FRAMEWORK_PARSE_PROTO_MSG_H_

#include "pc/util.h"
#include "ad_msg.h"
#include "chassis.pb.h"
#include "chassis_ctl_cmd.pb.h"
#include "writing_process_info.pb.h"
#include "gnss.pb.h"
#include "imu.pb.h"
#include "map_localization.pb.h"
#include "middle_layer.pb.h"
#include "scene_story.pb.h"
#include "special_chassis_info.pb.h"

namespace phoenix {
namespace framework {

class ParseProtoMsg {
 public:
  bool EncodeGnssMessage(const ad_msg::Gnss& msg,
                         msg::localization::Gnss* const data_out);
  bool DecodeGnssMessage(const Char_t* msg, Int32_t msg_len,
                         ad_msg::Gnss* data_out);

  bool EncodeImuMessage(const ad_msg::Imu& msg,
                        msg::localization::Imu* const data_out);
  bool DecodeImuMessage(const Char_t* msg, Int32_t msg_len,
                        ad_msg::Imu* data_out);

  bool EncodeWritingProcessInfoMessage(const ad_msg::WritingWordInfo& msg,std::string& str);
  bool DecodeWritingProcessInfoMessage(const std::string str,
                        ad_msg::WritingWordInfo* data_out);


  bool EncodeRtkResultMessage(const ad_msg::RtkResult& msg,
                              middle_layer_pb::RtkResult* const data_out);
  bool DecodeRtkResultMessage(const Char_t* msg, Int32_t msg_len,
                              ad_msg::RtkResult* data_out);

  bool EncodeLaserResultMessage(const ad_msg::LaserResult& msg,
                                middle_layer_pb::LaserResult* const data_out);
  bool DecodeLaserResultMessage(const Char_t* msg, Int32_t msg_len,
                                ad_msg::LaserResult* data_out);

  bool EncodeChassisMessage(const ad_msg::Chassis& msg,
                            msg::control::Chassis* const data_out);
  bool DecodeChassisMessage(const Char_t* msg, Int32_t msg_len,
                            ad_msg::Chassis* data_out);

  bool EncodeSpecialChassisInfoMessage(
      const ad_msg::SpecialChassisInfo& msg,
      msg::control::SpecialChassisInfo* const data_out);
  bool DecodeSpecialChassisInfoMessage(const Char_t* msg, Int32_t msg_len,
                                       ad_msg::SpecialChassisInfo* data_out);

  bool EncodeChassisCtlCmdMessage(const ad_msg::ChassisCtlCmd& msg,
                                  msg::control::ChassisCtlCmd* const data_out);
  bool DecodeChassisCtlCmdMessage(const Char_t* msg, Int32_t msg_len,
                                  ad_msg::ChassisCtlCmd* data_out);

  bool DecodeSceneStoryControlLine(const msg::routing::ControlLine& line_in,
                                   ad_msg::SceneStoryControlLine* line_out);
  bool DecodeSceneStoryCondition(const msg::routing::Condition& cond_in,
                                 ad_msg::SceneStoryCondition* cond_out);
  bool DecodeSceneStoryAction(const msg::routing::Action& action_in,
                              ad_msg::SceneStoryAction* action_out);
  bool DecodeSceneStorysMessage(const Char_t* msg, Int32_t msg_len,
                                ad_msg::SceneStoryList* data_out);

  bool DecodeMapLocalizationMessage(const Char_t* msg, Int32_t msg_len,
                                    ad_msg::MapLocalization* data_out);
};

}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_PARSE_PROTO_MSG_H_
