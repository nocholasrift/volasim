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

// Converts a MultiDOFJointTrajectory message into a trajectory_t.
// Validates strictly increasing time_from_start. Computes jerk from acc.
// Returns false and leaves `traj` unchanged on empty or invalid input.
template <typename TrajMsg, typename TimeGetter>
bool from_multidof_trajectory(const TrajMsg& msg, trajectory_t& traj,
                              TimeGetter get_time) {
  if (msg.points.empty()) {
    return false;
  }

  trajectory_t result;
  result.states.resize(msg.points.size());

  for (size_t i = 0; i < msg.points.size(); ++i) {
    from_multidof_point(msg.points[i], result.states[i]);
    result.states[i].time = get_time(msg.points[i]);
  }

  for (size_t i = 1; i < result.states.size(); ++i) {
    if (result.states[i].time <= result.states[i - 1].time) {
      return false;
    }
  }

  compute_jerk(result);
  traj = std::move(result);
  return true;
}

// Converts a trajectory_t into a MultiDOFJointTrajectory message.
// Caller sets msg.header before calling; this fills joint_names and points.
template <typename TrajMsg, typename DurationSetter>
void to_multidof_trajectory(const trajectory_t& traj, TrajMsg& msg,
                            DurationSetter set_duration) {
  msg.joint_names = {"base_link"};
  msg.points.reserve(traj.states.size());
  for (const auto& s : traj.states) {
    auto& pt = msg.points.emplace_back();
    to_multidof_point(s, pt);
    set_duration(pt, s.time);
  }
}

}  // namespace vola

#endif
