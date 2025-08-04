#ifndef ZMQ_SERVER_H
#define ZMQ_SERVER_H

#include <volasim/types.h>

#include <zmq.hpp>

#include <string_view>
#include <vector>

class ZMQServer {
 public:
  static ZMQServer& getInstance() {
    static ZMQServer instance;
    return instance;
  }

  void publishInfo(const std::string& sim_state);

  bool receiveInfo(std::string& input_buffer);

 private:
  ZMQServer();

  zmq::context_t context_;

  zmq::socket_t publisher_;
  zmq::socket_t subscriber_;
};

#endif
