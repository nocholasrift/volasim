#ifndef DEPTH_FRAME_H
#define DEPTH_FRAME_H

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

// One depth frame ready for the wire: the topic, the serialized DepthCamera
// header, and the raw uint16 millimetre payload. Produced on the render thread
// after a PBO readback completes, consumed on the comms thread.
struct DepthFrame {
  std::string           topic;
  std::string           header;    // serialized volasim_msgs::DepthCamera
  std::vector<uint16_t> depth_mm;  // row-major, width * height
};

// Hands depth frames from the render thread to the comms thread, keeping only
// the newest frame per sensor (drop-old): if comms lags, a stale cloud is
// overwritten rather than queued, matching the cloud socket's low send-HWM. The
// key identifies a sensor uniquely (drone id and sensor id packed together).
class CloudHandoff {
 public:
  void publish(uint64_t sensor_key, DepthFrame&& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_[sensor_key] = std::move(frame);
  }

  // Moves out every pending frame, leaving the handoff empty.
  std::vector<DepthFrame> drain() {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<DepthFrame> out;
    out.reserve(latest_.size());
    for (auto& [key, frame] : latest_) {
      out.push_back(std::move(frame));
    }
    latest_.clear();
    return out;
  }

 private:
  std::mutex                               mutex_;
  std::unordered_map<uint64_t, DepthFrame> latest_;
};

#endif
