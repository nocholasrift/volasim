#include <volasim/comms/zmq_server.h>
#include <zmq.hpp>

ZMQServer::ZMQServer() {
  context_ = zmq::context_t(1);
  publisher_ = zmq::socket_t(context_, zmq::socket_type::pub);
  subscriber_ = zmq::socket_t(context_, zmq::socket_type::sub);

  try {
    publisher_.bind("tcp://*:5556");
    subscriber_.connect("tcp://localhost:5557");
    subscriber_.set(zmq::sockopt::subscribe, "");
  } catch (const zmq::error_t& e) {
    throw std::runtime_error("Failed to initialize ZMQ server: " +
                             std::string(e.what()));
  }
}

void ZMQServer::publishInfo(const std::string& sim_state) {
  zmq::message_t message(sim_state.size());
  memcpy(message.data(), sim_state.data(), sim_state.size());
  publisher_.send(message, zmq::send_flags::none);
}

bool ZMQServer::receiveInfo(std::string& input_buffer) {
  zmq::message_t msg;
  auto result = subscriber_.recv(msg, zmq::recv_flags::dontwait);
  if (result) {
    input_buffer.assign(static_cast<char*>(msg.data()), msg.size());
    return true;
  }

  return false;
}
