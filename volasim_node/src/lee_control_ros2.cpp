#include "lee_controller_ros2.h"

#include <cmath>

LeeControlNode::LeeControlNode() : Node("lee_controller") {

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 1,
      std::bind(&LeeControlNode::odom_cb, this, std::placeholders::_1));

  desired_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "command_pos", 1,
      std::bind(&LeeControlNode::position_cb, this, std::placeholders::_1));

  cmd_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>("command", 10);

  control_loop_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(10),
                              std::bind(&LeeControlNode::control_loop, this));

  initialized_ = false;
  state_set_ = false;

  params_["kp"] = 69.44;
  params_["kv"] = 24.304;
  params_["kR"] = 13.81;
  params_["kw"] = 2.54;
  params_["mass"] = 4.34;
  params_["length"] = 0.315;
  params_["c_torque"] = 8.004e-4;
  params_["j0"] = 0.0820;
  params_["j1"] = 0.0845;
  params_["j2"] = 0.1377;

  controller_.loadParams(params_);

  conv_mat_ << 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1;

  double l = params_["length"];
  conv_mat_.row(1) *= l / std::sqrt(2);
  conv_mat_.row(2) *= l / std::sqrt(2);
  conv_mat_.row(3) *= params_["c_torque"];

  conv_mat_inverse_ = conv_mat_.inverse();

  RCLCPP_INFO(this->get_logger(), "Lee Controller Node initialized.");
}

void LeeControlNode::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  state_.pos =
      Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
  state_.vel =
      Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                      msg->twist.twist.linear.z);
  state_.w =
      Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                      msg->twist.twist.angular.z);

  Eigen::Quaterniond quat(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  state_.rot = quat.toRotationMatrix();
  initialized_ = true;
}

void LeeControlNode::position_cb(
    const geometry_msgs::msg::Point::SharedPtr msg) {

  // if (!initialized_)
  //   return;

  desired_state_.reset();
  desired_state_.pos = Eigen::Vector3d(msg->x, msg->y, msg->z);
  traj_ = generateTraj(state_, desired_state_, 2.0);
  state_set_ = true;
  start_ = this->now();
}

void LeeControlNode::control_loop() {
  if (!initialized_ || !state_set_)
    return;
  if (traj_.points.empty())
    return;

  double t = (this->now() - start_).seconds();
  size_t ind = 0;
  for (size_t i = 0; i < traj_.points.size(); ++i) {
    if (t < traj_.points[i].time_from_start.sec +
                traj_.points[i].time_from_start.nanosec * 1e-9) {
      break;
    }
    ind = i;
  }
  ind = std::min(ind, traj_.points.size() - 1);

  auto pt = traj_.points[ind];

  vola::state_t desired_s;
  desired_s.pos =
      Eigen::Vector3d(pt.positions[0], pt.positions[1], pt.positions[2]);
  desired_s.vel =
      Eigen::Vector3d(pt.velocities[0], pt.velocities[1], pt.velocities[2]);
  desired_s.acc = Eigen::Vector3d(pt.accelerations[0], pt.accelerations[1],
                                  pt.accelerations[2]);
  desired_s.jerk = Eigen::Vector3d(pt.effort[0], pt.effort[1], pt.effort[2]);

  Eigen::Vector4d cmd = controller_.computeControls(state_, desired_s);

  std_msgs::msg::Float32MultiArray msg;
  msg.data = {static_cast<float>(cmd[0]), static_cast<float>(cmd[1]),
              static_cast<float>(cmd[2]), static_cast<float>(cmd[3])};

  cmd_pub_->publish(msg);
}

trajectory_msgs::msg::JointTrajectory LeeControlNode::generateTraj(
    const vola::state_t& start, const vola::state_t& end, double T) {
  Eigen::MatrixXd t_mat(6, 6);
  t_mat.row(0) << 0., 0., 0., 0., 0., 1.;
  t_mat.row(1) << T * T * T * T * T, T * T * T * T, T * T * T, T * T, T, 1;
  t_mat.row(2) << 0., 0., 0., 0., 1., 0.;
  t_mat.row(3) << 5 * T * T * T * T, 4 * T * T * T, 3 * T * T, 2 * T, 1., 0.;
  t_mat.row(4) << 0., 0., 0., 2., 0., 0.;
  t_mat.row(5) << 20 * T * T * T, 12 * T * T, 6 * T, 2, 0., 0.;

  Eigen::MatrixXd t_mat_inv = t_mat.inverse();

  Eigen::VectorXd bounds(6);
  std::array<Eigen::VectorXd, 3> coeffs;
  for (int i = 0; i < 3; ++i) {
    bounds << start.pos(i), end.pos(i), start.vel(i), end.vel(i), 0., 0.;
    coeffs[i] = t_mat_inv * bounds;
  }

  auto eval_poly = [](const Eigen::VectorXd& c, double t) {
    double ret = 0;
    double t_pow = 1;
    for (int i = 0; i < c.size(); ++i) {
      ret += c[c.size() - 1 - i] * t_pow;
      t_pow *= t;
    }
    return ret;
  };

  trajectory_msgs::msg::JointTrajectory traj;
  size_t sz = static_cast<size_t>(T / 0.05) + 1;
  double t = 0.;
  traj.points.reserve(sz);
  for (size_t i = 0; i < sz; ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    for (int j = 0; j < 3; ++j) {
      pt.positions.push_back(eval_poly(coeffs[j], t));

      Eigen::VectorXd v_coeffs(5);
      v_coeffs << 5 * coeffs[j][0], 4 * coeffs[j][1], 3 * coeffs[j][2],
          2 * coeffs[j][3], coeffs[j][4];
      pt.velocities.push_back(eval_poly(v_coeffs, t));

      Eigen::VectorXd a_coeffs(4);
      a_coeffs << 20 * coeffs[j][0], 12 * coeffs[j][1], 6 * coeffs[j][2],
          2 * coeffs[j][3];
      pt.accelerations.push_back(eval_poly(a_coeffs, t));

      Eigen::VectorXd j_coeffs(3);
      j_coeffs << 60 * coeffs[j][0], 24 * coeffs[j][1], 6 * coeffs[j][2];
      pt.effort.push_back(eval_poly(j_coeffs, t));
    }
    pt.time_from_start = rclcpp::Duration::from_seconds(t);
    traj.points.push_back(pt);
    t += 0.05;
  }

  return traj;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeeControlNode>());
  rclcpp::shutdown();
  return 0;
}
