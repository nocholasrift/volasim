#ifndef VOLASIM_NODE_LEE_CONTROLLER_H
#define VOLASIM_NODE_LEE_CONTROLLER_H

#include "types.h"

#include <string_view>
#include <unordered_map>

/*
 * This code is based on the paper Geometric Tracking Control of a Quadrotor UAV on SE(3)
 */

namespace vola {

class LeeController {
 public:
  LeeController();

  ~LeeController();

  void loadParams(std::unordered_map<std::string_view, double>& params);

  void computeControls(const state_t& state, const state_t& desired_state);

 private:
  state_t state_;

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
