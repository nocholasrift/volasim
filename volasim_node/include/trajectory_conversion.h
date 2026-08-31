#ifndef VOLASIM_NODE_TRAJECTORY_CONVERSION_H
#define VOLASIM_NODE_TRAJECTORY_CONVERSION_H

#include "types.h"

namespace vola {

// Extracts pos/vel/acc from a MultiDOFJointTrajectoryPoint into a state_t.
// Works with both ROS1 and ROS2 message types — field names are identical.
template <typename PointT>
void from_multidof_point(const PointT& pt, state_t& s) {
  if (!pt.transforms.empty()) {
    s.pos = Eigen::Vector3d(pt.transforms[0].translation.x,
                            pt.transforms[0].translation.y,
                            pt.transforms[0].translation.z);
  }
  if (!pt.velocities.empty()) {
    s.vel =
        Eigen::Vector3d(pt.velocities[0].linear.x, pt.velocities[0].linear.y,
                        pt.velocities[0].linear.z);
  }
  if (!pt.accelerations.empty()) {
    s.acc = Eigen::Vector3d(pt.accelerations[0].linear.x,
                            pt.accelerations[0].linear.y,
                            pt.accelerations[0].linear.z);
  }
}

// Fills a MultiDOFJointTrajectoryPoint from a state_t.
// Uses emplace_back on the point's vectors, so call on a fresh point.
template <typename PointT>
void to_multidof_point(const state_t& s, PointT& pt) {
  auto& tf         = pt.transforms.emplace_back();
  tf.translation.x = s.pos.x();
  tf.translation.y = s.pos.y();
  tf.translation.z = s.pos.z();
  tf.rotation.w    = 1.0;

  auto& vel    = pt.velocities.emplace_back();
  vel.linear.x = s.vel.x();
  vel.linear.y = s.vel.y();
  vel.linear.z = s.vel.z();

  auto& acc    = pt.accelerations.emplace_back();
  acc.linear.x = s.acc.x();
  acc.linear.y = s.acc.y();
  acc.linear.z = s.acc.z();
}

}  // namespace vola

#endif
