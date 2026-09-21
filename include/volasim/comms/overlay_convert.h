#ifndef VOLASIM_OVERLAY_CONVERT_H
#define VOLASIM_OVERLAY_CONVERT_H

#include <volasim/simulation/tube_shape.h>

#include <volasim_msgs/Trajectory.pb.h>

#include <glm/glm.hpp>

namespace volasim::overlay {

inline TrajectoryData fromProto(const volasim_msgs::Trajectory& proto) {
  TrajectoryData data;
  data.points.reserve(proto.points_size());

  for (const auto& pt : proto.points()) {
    data.points.emplace_back(static_cast<float>(pt.pos().x()),
                             static_cast<float>(pt.pos().y()),
                             static_cast<float>(pt.pos().z()));
  }

  if (proto.has_header()) {
    data.frame_id = proto.header().frame_id();
  }

  return data;
}

}  // namespace volasim::overlay

#endif
