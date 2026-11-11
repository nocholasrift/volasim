#ifndef VOLASIM_NODE_LEE_CONTROLLER_H
#define VOLASIM_NODE_LEE_CONTROLLER_H

#include "types.h"

#include <string_view>
#include <unordered_map>

/*
 * This code is based on the paper Geometric Tracking Control of a Quadrotor UAV on SE(3)
 */

namespace vola {

// World frame unless marked body.
struct lee_debug_t {
  Eigen::Vector3d ep        = Eigen::Vector3d::Zero();  // state - desired
  Eigen::Vector3d ev        = Eigen::Vector3d::Zero();  // state - desired
  Eigen::Vector3d pid_term  = Eigen::Vector3d::Zero();
  double          pid_norm  = 0.;
  double          fz        = 0.;
  bool            cutoff    = false;
  Eigen::Vector3d b3d       = Eigen::Vector3d::Zero();
  bool            flipped   = false;
  double          psi       = 0.;
  Eigen::Vector3d h_w       = Eigen::Vector3d::Zero();
  Eigen::Vector3d desired_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d eR        = Eigen::Vector3d::Zero();  // body
  Eigen::Vector3d ew        = Eigen::Vector3d::Zero();  // body
  Eigen::Vector3d torque_R  = Eigen::Vector3d::Zero();  // body
  Eigen::Vector3d torque_w  = Eigen::Vector3d::Zero();  // body
};

class LeeController {
 public:
  LeeController();

  ~LeeController();

  void loadParams(const std::unordered_map<std::string_view, double>& params);

  Eigen::Vector4d computeControls(const state_t& state,
                                  const state_t& desired_state);

  const lee_debug_t& debug() const { return debug_; }

 private:
  double orientationError(const Eigen::Matrix3d& m1, const Eigen::Matrix3d& m2);
  state_t     state_;
  lee_debug_t debug_;

  double kp_ = 1.;
  double kv_ = 1.;
  double kR_ = 1.;
  double kw_ = 1.;

  Eigen::Matrix3d J_ = Eigen::Matrix3d::Zero();

  double mass_ = 0.;

  static constexpr double g = 9.81;

  const Eigen::Vector3d e1_ = Eigen::Vector3d(1, 0, 0);
  const Eigen::Vector3d e2_ = Eigen::Vector3d(0, 1, 0);
  const Eigen::Vector3d e3_ = Eigen::Vector3d(0, 0, 1);
};

}  // namespace vola

#endif
