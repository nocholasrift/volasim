#ifndef VOLASIM_COMMS_TOPICS_H
#define VOLASIM_COMMS_TOPICS_H

#include <cstdint>
#include <string>

// Topic strings are built in one place so the wire contract has a single source
// of truth and can be unit-tested without pulling in ZMQ.
//
// Messages are multipart. The first frame is always the topic, which is what
// SUB-side prefix filtering matches on, so subscribing to "drone/7/" delivers
// every stream for drone 7 and nothing else.
//
//   state: [topic]["drone/<id>/state"] [DroneState]
//   depth: [topic]["drone/<id>/depth"] [DepthCamera header] [raw uint16 payload]
namespace volasim::topics {

inline std::string drone(std::uint32_t drone_id) {
  return "drone/" + std::to_string(drone_id);
}

inline std::string state(std::uint32_t drone_id) {
  return drone(drone_id) + "/state";
}

inline std::string depth(std::uint32_t drone_id) {
  return drone(drone_id) + "/depth";
}

}  // namespace volasim::topics

#endif
