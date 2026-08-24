#include <volasim/comms/zmq_server.h>
#include <zmq.hpp>

ZMQServer::ZMQServer() {
  context_         = zmq::context_t(1);
  state_publisher_ = zmq::socket_t(context_, zmq::socket_type::pub);
  cloud_publisher_ = zmq::socket_t(context_, zmq::socket_type::pub);
  subscriber_      = zmq::socket_t(context_, zmq::socket_type::sub);

  try {
    // The ROS bridge is the consumer and always runs on the same host, so it
    // connects over ipc; tcp is bound too so ad-hoc tools (a zmq sniffer, a
    // remote dashboard) can still reach the fast stream.
    state_publisher_.bind("ipc:///tmp/volasim_state");
    state_publisher_.bind("tcp://*:5556");

    // Small queue so a slow cloud consumer drops stale frames instead of
    // building lag; must be set before bind() to take effect.
    cloud_publisher_.set(zmq::sockopt::sndhwm, 2);
    // ipc for same-host native consumers; tcp so the ROS bridge can reach the
    // cloud stream from inside a container, mirroring the state publisher.
    cloud_publisher_.bind("ipc:///tmp/volasim_cloud");
    cloud_publisher_.bind("tcp://*:5559");

    subscriber_.connect("tcp://localhost:5557");
    subscriber_.set(zmq::sockopt::subscribe, "");
  } catch (const zmq::error_t& e) {
    throw std::runtime_error("Failed to initialize ZMQ server: " +
                             std::string(e.what()));
  }
}

void ZMQServer::publishState(const std::string& topic,
                             const std::string& state_bytes) {
  state_publisher_.send(zmq::buffer(topic), zmq::send_flags::sndmore);
  state_publisher_.send(zmq::buffer(state_bytes), zmq::send_flags::none);
}

void ZMQServer::publishSensor(const std::string& topic,
                              const std::string& header_bytes,
                              zmq::message_t&&   payload) {
  cloud_publisher_.send(zmq::buffer(topic), zmq::send_flags::sndmore);
  cloud_publisher_.send(zmq::buffer(header_bytes), zmq::send_flags::sndmore);
  cloud_publisher_.send(std::move(payload), zmq::send_flags::none);
}

bool ZMQServer::receiveInfo(std::string& input_buffer) {
  zmq::message_t msg;
  auto           result = subscriber_.recv(msg, zmq::recv_flags::dontwait);
  if (result) {
    input_buffer.assign(static_cast<char*>(msg.data()), msg.size());
    return true;
  }

  return false;
}
