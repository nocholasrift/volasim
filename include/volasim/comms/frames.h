#ifndef VOLASIM_COMMS_FRAMES_H
#define VOLASIM_COMMS_FRAMES_H

#include <cstdint>
#include <string>

// TF frame names, built in one place so the sim is the single source of truth
// for the transform tree and the bridge never has to know the convention (it
// reads frame ids straight off the wire). Namespaced per drone in the ROS
// tf_prefix style so multiple drones share no frames:
//
//   drone_<id>/odom  ->  drone_<id>/base_link  ->  drone_<id>/depth_<sensor>
namespace volasim::frames {

// Folds an arbitrary sensor name into a token that is legal as both a tf frame
// component and a ROS topic component: ROS rejects anything outside
// [A-Za-z0-9_] (a hyphen, dot or space aborts create_publisher and kills the
// bridge). Applied at name assignment, before the registry de-dupes, so two
// names that fold together (e.g. "front-depth" and "front_depth") collide and
// get bumped apart rather than racing for one topic.
inline std::string toToken(const std::string& name) {
  std::string out;
  out.reserve(name.size());
  for (char c : name) {
    const bool legal = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                       (c >= '0' && c <= '9') || c == '_';
    out.push_back(legal ? c : '_');
  }
  return out;
}

inline std::string drone(std::uint32_t drone_id) {
  return "drone_" + std::to_string(drone_id);
}

inline std::string odom(std::uint32_t drone_id) {
  return drone(drone_id) + "/odom";
}

inline std::string baseLink(std::uint32_t drone_id) {
  return drone(drone_id) + "/base_link";
}

// A sensor's link frame, named by its XML name under the drone namespace. The
// drone prefix already disambiguates sensors across drones; the parser ensures
// names are unique within a drone.
inline std::string sensor(std::uint32_t drone_id, const std::string& name) {
  return drone(drone_id) + "/" + name;
}

// Optical frame for a sensor: same origin as its link frame but with the REP-145
// optical convention (x-right, y-down, z-forward). Depth clouds are expressed in
// this frame; the link -> optical edge carries the fixed rotation.
inline std::string sensorOptical(std::uint32_t      drone_id,
                                 const std::string& name) {
  return sensor(drone_id, name) + "_optical";
}

}  // namespace volasim::frames

#endif
