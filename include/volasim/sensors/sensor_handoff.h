#ifndef SENSOR_HANDOFF_H
#define SENSOR_HANDOFF_H

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

struct SensorFrame {
  std::string          topic;
  std::string          header;
  std::vector<uint8_t> payload;
};

// Keeps only the newest frame per sensor so a slow comms thread drops stale
// readings instead of building a backlog.
class SensorHandoff {
 public:
  void publish(const std::string& sensor_key, SensorFrame&& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_[sensor_key] = std::move(frame);
  }

  std::vector<SensorFrame> drain() {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<SensorFrame> out;
    out.reserve(latest_.size());
    for (auto& [key, frame] : latest_) {
      out.push_back(std::move(frame));
    }
    latest_.clear();
    return out;
  }

 private:
  std::mutex                                   mutex_;
  std::unordered_map<std::string, SensorFrame> latest_;
};

#endif
