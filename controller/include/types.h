#ifndef VOLASIM_NODE_TYPES_H
#define VOLASIM_NODE_TYPES_H

#include <Eigen/Dense>
#include <vector>

namespace vola {

struct state_t {
  Eigen::Vector3d pos  = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel  = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc  = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rot  = Eigen::Matrix3d::Identity();

  Eigen::Vector3d w     = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_dot = Eigen::Vector3d::Zero();
  double          yaw   = 0.;
  double          time  = 0.;

  void reset() {
    pos  = Eigen::Vector3d::Zero();
    vel  = Eigen::Vector3d::Zero();
    acc  = Eigen::Vector3d::Zero();
    jerk = Eigen::Vector3d::Zero();
    rot  = Eigen::Matrix3d::Identity();

    w     = Eigen::Vector3d::Zero();
    w_dot = Eigen::Vector3d::Zero();

    time = 0;
    yaw  = 0;
  }
};

struct trajectory_t {
  std::vector<state_t> states;

  const state_t& at_time(double t) const {
    if (states.empty()) {
      static const state_t zero;
      return zero;
    }
    for (size_t i = 1; i < states.size(); ++i) {
      if (states[i].time > t) {
        return states[i - 1];
      }
    }
    return states.back();
  }
};

inline void compute_jerk(trajectory_t& traj) {
  if (traj.states.size() < 2) {
    return;
  }
  for (size_t i = 0; i < traj.states.size() - 1; ++i) {
    double dt = traj.states[i + 1].time - traj.states[i].time;
    if (dt > 0) {
      traj.states[i].jerk = (traj.states[i + 1].acc - traj.states[i].acc) / dt;
    }
  }
  traj.states.back().jerk = traj.states[traj.states.size() - 2].jerk;
}

}  // namespace vola

#endif
