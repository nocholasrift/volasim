#ifndef ZMQ_SERVER_H
#define ZMQ_SERVER_H

#include <zmq.hpp>

class ZMQServer {
 public:
  static ZMQServer& getInstance() {
    static ZMQServer instance;
    return instance;
  }

 private:
  ZMQServer();

  zmq::context_t context_;

  zmq::socket_t publisher_;
  zmq::socket_t subscriber_;
};

#endif
