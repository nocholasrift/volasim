#include "lee_controller.h"
#include "matrix_utils.h"

#include <iostream>

namespace vola {

LeeController::LeeController() {}

LeeController::~LeeController() {}

void LeeController::loadParams(
    std::unordered_map<std::string_view, double>& params) {
  kp_ = params["kp"];
  kv_ = params["kv"];
  kR_ = params["kR"];
  kw_ = params["kw"];

  mass_ = params["mass"];

  // can change this later, but must drones fulfill this form
  J_(0, 0) = params["j0"];
  J_(1, 1) = params["j1"];
  J_(2, 2) = params["j2"];
}

Eigen::Vector4d LeeController::computeControls(const state_t& state,
                                               const state_t& desired_state) {

  Eigen::Vector3d ep = state.pos - desired_state.pos;
  Eigen::Vector3d ev = state.vel - desired_state.vel;

  Eigen::Vector3d pid_term =
      (-kp_ * ep - kv_ * ev + mass_ * g * e3_ + mass_ * desired_state.acc);

  double fz = pid_term.dot(state.rot * e3_);

  double pid_term_norm = pid_term.norm();

  if (pid_term_norm == 0) {
    throw std::runtime_error(
        "Controller error: PID term norm is zero, cannot compute thrust "
        "direction");
  }

  // z_dw stores the desired thrust direction
  Eigen::Vector3d z_dw = pid_term / pid_term_norm;

  Eigen::Vector3d yaw_vec(cos(desired_state.yaw), sin(desired_state.yaw), 0);

  Eigen::Vector3d zdw_x_yaw_vec = z_dw.cross(yaw_vec);
  Eigen::Vector3d R_dw_y = zdw_x_yaw_vec / zdw_x_yaw_vec.norm();
  Eigen::Vector3d zdw_yv_x_zdw = zdw_x_yaw_vec.cross(z_dw);
  Eigen::Vector3d R_dw_x = zdw_yv_x_zdw / zdw_yv_x_zdw.norm();

  Eigen::Matrix3d R_dw;
  R_dw << R_dw_x, R_dw_y, z_dw;

  Eigen::Matrix3d R_bw_inv = state.rot.transpose();
  Eigen::Vector3d eR =
      .5 * utils::vee(R_dw.transpose() * state.rot - R_bw_inv * R_dw);
  Eigen::Vector3d ew = state.w - R_bw_inv * desired_state.rot * desired_state.w;

  Eigen::Vector3d diff_dyna =
      utils::hat(state.w) * R_bw_inv * R_dw * desired_state.w -
      R_bw_inv * R_dw * desired_state.w_dot;

  Eigen::Vector3d torque =
      -kR_ * eR - kw_ * ew + state.w.cross(J_ * state.w) - J_ * diff_dyna;

  // std::cout << "kr * er - kw * ew: " << (-kR_ * eR - kw_ * ew).transpose()
  //           << "\n";

  Eigen::Vector4d cmd;
  cmd[0] = fz;
  cmd.tail(3) = torque;

  return cmd;
}

}  // namespace vola
