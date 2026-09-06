#include "lee_controller.h"
#include "matrix_utils.h"

#include <iostream>

namespace vola {

LeeController::LeeController() = default;

LeeController::~LeeController() = default;

void LeeController::loadParams(
    const std::unordered_map<std::string_view, double>& params) {
  double m  = params.at("mass");
  double j0 = params.at("j0");
  double j1 = params.at("j1");
  double j2 = params.at("j2");

  if (m <= 0 || j0 <= 0 || j1 <= 0 || j2 <= 0) {
    throw std::runtime_error("mass and inertia values must be positive");
  }

  kp_   = params.at("kp");
  kv_   = params.at("kv");
  kR_   = params.at("kR");
  kw_   = params.at("kw");
  mass_ = m;

  J_(0, 0) = j0;
  J_(1, 1) = j1;
  J_(2, 2) = j2;
}

Eigen::Vector4d LeeController::computeControls(const state_t& state,
                                               const state_t& desired_state) {

  Eigen::Vector3d ep = state.pos - desired_state.pos;
  Eigen::Vector3d ev = state.vel - desired_state.vel;

  Eigen::Vector3d pid_term =
      (-kp_ * ep - kv_ * ev + mass_ * g * e3_ + mass_ * desired_state.acc);

  double pid_term_norm = pid_term.norm();

  if (pid_term_norm < 1e-8) {
    throw std::runtime_error(
        "Controller error: PID term norm is zero, cannot compute thrust "
        "direction");
  }

  Eigen::Vector3d z_b = desired_state.acc + Eigen::Vector3d(0, 0, 9.81);
  z_b.normalize();

  double fz = pid_term.dot(z_b);

  if (fz < 1e-8) {
    return Eigen::Vector4d::Zero();
  }

  Eigen::Vector3d b3d = pid_term / pid_term.norm();
  Eigen::Vector3d x_c(cos(desired_state.yaw), sin(desired_state.yaw), 0);

  Eigen::Vector3d b2d = b3d.cross(x_c);
  if (b2d.norm() < 1e-8) {
    throw std::runtime_error("yaw and thrust vectors are parallel.");
  }
  b2d.normalize();

  Eigen::Vector3d b1d = b2d.cross(b3d);

  Eigen::Matrix3d R_dw;
  R_dw.col(0) = b1d;
  R_dw.col(1) = b2d;
  R_dw.col(2) = b3d;

  Eigen::Matrix3d R_dw2;
  R_dw2.col(0) = -b1d;
  R_dw2.col(1) = -b2d;
  R_dw2.col(2) = b3d;

  // error is locally positive semi-definite within region where R_d
  // and R are les than 180 degrees apart... if we are close to this
  // the controller has already lost the drone :)
  if (orientationError(R_dw, state.rot) > orientationError(R_dw2, state.rot)) {
    R_dw = R_dw2;
    b1d  = R_dw.col(0);
    b2d  = R_dw.col(1);
  }

  Eigen::Vector3d h_w =
      (mass_ / fz) * (desired_state.jerk - (z_b.dot(desired_state.jerk)) * z_b);

  double w_p = -h_w.dot(b2d);
  double w_q = h_w.dot(b1d);
  double w_r = 0.;  // come back and implement yaw control at some point

  Eigen::Vector3d desired_w = w_p * b1d + w_q * b2d + w_r * b3d;

  Eigen::Matrix3d R_bw_inv = state.rot.transpose();
  Eigen::Vector3d eR =
      .5 * utils::vee(R_dw.transpose() * state.rot - R_bw_inv * R_dw);

  Eigen::Vector3d ew = state.w - desired_w;

  Eigen::Vector3d torque = -kR_ * eR - kw_ * ew;

  Eigen::Vector4d cmd;
  cmd[0]      = fz;
  cmd.tail(3) = torque;

  return cmd;
}

double LeeController::orientationError(const Eigen::Matrix3d& m1,
                                       const Eigen::Matrix3d& m2) {
  return (Eigen::Matrix3d::Identity() - m1.transpose() * m2).trace() / 2;
}

}  // namespace vola
