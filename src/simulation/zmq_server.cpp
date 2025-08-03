#include <volasim/simulation/zmq_server.h>

ZMQServer::ZMQServer() {
  context_ = zmq::context_t(1);
  publisher_ = zmq::socket_t(context_, zmq::socket_type::pub);
  subscriber_ = zmq::socket_t(context_, zmq::socket_type::sub);
}
