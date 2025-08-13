#ifndef VOLASIM_NODE_TYPES_H
#define VOLASIM_NODE_TYPES_H

#include <Eigen/Dense>

namespace vola {

struct state_t {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();

  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_dot = Eigen::Vector3d::Zero();
  double yaw = 0.;
};

}  // namespace vola

#endif
