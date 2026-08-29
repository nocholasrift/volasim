#ifndef VOLASIM_COMMS_FRAME_REGISTRY_H
#define VOLASIM_COMMS_FRAME_REGISTRY_H

#include <volasim/comms/frames.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace volasim::frames {

// Hands out unique tf frame names per drone. Uniqueness is enforced over the
// full generated frame ids — both a sensor's link and optical frame — and the
// root frames are pre-reserved. So a sensor named "base_link" or "odom", or one
// whose name would land on another sensor's optical frame (e.g. "foo" then
// "foo_optical"), gets bumped instead of shadowing an existing frame.
class Registry {
 public:
  struct SensorFrames {
    std::string name;  // the (possibly bumped) name both frames were built from
    std::string link;
    std::string optical;
  };

  // Reserve a drone's root frames so no sensor can take odom or base_link.
  void reserveRoots(std::uint32_t drone_id) {
    auto& set = taken_[drone_id];
    set.insert(odom(drone_id));
    set.insert(baseLink(drone_id));
  }

  // Assign a sensor's link/optical frames from a base name, bumping a numeric
  // suffix until both frames are free, then reserving both.
  SensorFrames assignSensor(std::uint32_t drone_id, const std::string& base) {
    auto&             set   = taken_[drone_id];
    const std::string token = toToken(base);
    std::string       name  = token;
    for (std::uint32_t n = 1;; ++n) {
      SensorFrames frames{name, sensor(drone_id, name),
                          sensorOptical(drone_id, name)};
      if (set.count(frames.link) == 0 && set.count(frames.optical) == 0) {
        set.insert(frames.link);
        set.insert(frames.optical);
        return frames;
      }
      name = token + "_" + std::to_string(n);
    }
  }

 private:
  std::unordered_map<std::uint32_t, std::unordered_set<std::string>> taken_;
};

}  // namespace volasim::frames

#endif
