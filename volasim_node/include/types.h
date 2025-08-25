#ifndef VOLASIM_NODE_TYPES_H
#define VOLASIM_NODE_TYPES_H

#include <Eigen/Dense>

namespace vola {

struct state_t {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();

  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_dot = Eigen::Vector3d::Zero();
  double yaw = 0.;

  void reset() {
    pos = Eigen::Vector3d::Zero();
    vel = Eigen::Vector3d::Zero();
    acc = Eigen::Vector3d::Zero();
    jerk = Eigen::Vector3d::Zero();
    rot = Eigen::Matrix3d::Identity();

    w = Eigen::Vector3d::Zero();
    w_dot = Eigen::Vector3d::Zero();
  }
};

}  // namespace vola

#endif
