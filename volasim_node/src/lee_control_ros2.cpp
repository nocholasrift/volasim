#include "lee_controller_ros2.h"
#include "trajectory_conversion.h"

#include <cmath>

LeeControlNode::LeeControlNode() : Node("lee_controller") {
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 1,
      std::bind(&LeeControlNode::odom_cb, this, std::placeholders::_1));

  traj_sub_ =
      this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
          "cmd_trajectory", 1,
          std::bind(&LeeControlNode::trajectory_cb, this,
                    std::placeholders::_1));

  cmd_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>("command", 10);

  control_loop_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(5),
                              std::bind(&LeeControlNode::control_loop, this));

  initialized_ = false;
  traj_set_    = false;

  params_["kp"]       = 3.5;
  params_["kv"]       = 2.1;
  params_["kR"]       = 1.0;
  params_["kw"]       = 0.1;
  params_["mass"]     = 0.68;
  params_["length"]   = 0.17;
  params_["c_torque"] = 0.016;
  params_["j0"]       = 0.007;
  params_["j1"]       = 0.007;
  params_["j2"]       = 0.012;

  controller_.loadParams(params_);

  conv_mat_ << 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1;

  double l          = params_["length"];
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

void LeeControlNode::trajectory_cb(
    const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg) {
  if (msg->points.empty()) {
    return;
  }

  active_traj_.states.resize(msg->points.size());
  for (size_t i = 0; i < msg->points.size(); ++i) {
    vola::from_multidof_point(msg->points[i], active_traj_.states[i]);
    active_traj_.states[i].time =
        rclcpp::Duration(msg->points[i].time_from_start).seconds();
  }

  vola::compute_jerk(active_traj_);

  start_    = this->now();
  traj_set_ = true;
}

void LeeControlNode::control_loop() {
  if (!initialized_ || !traj_set_) {
    return;
  }

  double          t         = (this->now() - start_).seconds();
  const auto&     desired_s = active_traj_.at_time(t);
  Eigen::Vector4d cmd       = controller_.computeControls(state_, desired_s);

  std_msgs::msg::Float32MultiArray msg;
  msg.data = {
      static_cast<float>(cmd[Motors::M1]), static_cast<float>(cmd[Motors::M2]),
      static_cast<float>(cmd[Motors::M3]), static_cast<float>(cmd[Motors::M4])};

  cmd_pub_->publish(msg);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeeControlNode>());
  rclcpp::shutdown();
  return 0;
}
