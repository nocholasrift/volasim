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

  // Fast per-drone state: [topic][serialized DroneState] on the low-latency
  // socket.
  void publishState(const std::string& topic, const std::string& state_bytes);

  // Bulk sensor data: [topic][serialized header][raw payload] on the cloud
  // socket. The payload is moved in so a large buffer reaches the wire without
  // an extra copy.
  void publishSensor(const std::string& topic, const std::string& header_bytes,
                     zmq::message_t&& payload);

  bool receiveInfo(std::string& input_buffer);

 private:
  ZMQServer();

  zmq::context_t context_;

  // Small, high-rate state (IMU, odom) is kept off the cloud socket so a
  // multi-MB frame never head-of-line-blocks it.
  zmq::socket_t state_publisher_;
  // Bulk sensor frames. A low send-HWM lets old frames drop under backpressure
  // rather than queueing — a lagging consumer gets the newest cloud, not a
  // growing backlog. (ZMQ_CONFLATE would do this too but only for single-part
  // messages, and these are multipart.)
  zmq::socket_t cloud_publisher_;
  zmq::socket_t subscriber_;
};

#endif
