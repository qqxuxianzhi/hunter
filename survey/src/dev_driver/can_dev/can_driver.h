#ifndef PHOENIX_CAN_DEV_CAN_DRIVER_H_
#define PHOENIX_CAN_DEV_CAN_DRIVER_H_


#include "utils/macros.h"
#include "utils/com_utils.h"

namespace phoenix {
namespace can_dev {

enum {
  CAN_BIT_RATE_INVALID = 0,
  CAN_BIT_RATE_250K,
  CAN_BIT_RATE_500K
};

struct CanChannelParam {
  Int32_t channel;
  Int32_t bit_rate;

  struct {
    Char_t ip_addr[32];
    Uint16_t port;
  } can_net;

  void Clear() {
    channel = -1;
    bit_rate = CAN_BIT_RATE_INVALID;
    common::com_memset(can_net.ip_addr, 0, sizeof(can_net.ip_addr));
    can_net.port = 0;
  }

  CanChannelParam() {
    Clear();
  }
};

struct CanFrame {
  Uint32_t id = 0;
  bool is_remote = false;
  bool is_extern = false;
  Uint64_t time_stamp = 0;
  Int32_t data_len;
  Uint8_t data[8] = {0};

  void Clear() {
    id = 0;
    is_remote = false;
    is_extern = false;
    time_stamp = 0;
    data_len = 0;
    common::com_memset(data, 0, sizeof(data));
  }

  CanFrame() {
    Clear();
  }
};


class CanDriver {
public:
  CanDriver();
  virtual ~CanDriver();

  virtual bool OpenChannel(const CanChannelParam& param);
  virtual bool CloseChannel();
  virtual Int32_t Send(const CanFrame* frame, Int32_t frame_num = 1);
  virtual Int32_t ReadWait(
      CanFrame* frame, Int32_t max_frame_num = 1, Uint32_t timeout_ms = 50);
};


}  // namespace can_dev
}  // namespace phoenix


#endif // PHOENIX_CAN_DEV_CAN_DRIVER_H_
