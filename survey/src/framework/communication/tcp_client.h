#ifndef PHOENIX_FRAMEWORK_TCP_CLIENT_H_
#define PHOENIX_FRAMEWORK_TCP_CLIENT_H_

#include <arpa/inet.h>

#include <functional>
#include <memory>

#include "os/mutex.h"
#include "os/thread.h"
#include "utils/macros.h"
namespace phoenix {
namespace framework {

typedef std::function<void(const Uint8_t *buf, const Int32_t buf_len,
                           const Int64_t receive_time)>
    TcpClientRecvMsgCallback;

class TcpClient {
 public:
  struct TcpClientParam {
    bool enable;
    Uint16_t server_port;
    Char_t server_addr[20];
    Uint64_t connect_timeout_ms;
    void Clear() {
      enable = false;
      server_port = 0;
      connect_timeout_ms = 1 * 1000;
      common::com_memset(server_addr, 0, sizeof(server_addr));
    }
    TcpClientParam() { Clear(); }
  };

  TcpClient();
  ~TcpClient();

  bool Start(const TcpClientParam &param);
  bool Stop();
  bool StartReceiving();
  void TheadReceivingMessages();
  void SetRecvMsgCallback(TcpClientRecvMsgCallback callback) {
    recv_msg_callback_ = std::move(callback);
  }
  Int32_t Send(const Uint8_t *buf, const Int32_t buf_len);

 private:
  TcpClientParam tcp_client_param_;
  Int32_t sock_fd_;
  Uint8_t receiving_msg_buf_[65536];
  bool receiving_flag_;
  common::os::ThreadFuncHelper<TcpClient> receiving_thread_func_helper_;
  common::os::Thread receiving_thread_;
  TcpClientRecvMsgCallback recv_msg_callback_;
  common::os::Mutex lock_sending_;
};

typedef std::shared_ptr<TcpClient> TcpClientPtr;

}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_TCP_CLIENT_H_
