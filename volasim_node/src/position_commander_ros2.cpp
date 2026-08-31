#include <minjerk_generator.h>
#include <trajectory_conversion.h>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

class PositionCommander : public rclcpp::Node {
 public:
  PositionCommander();

 private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void command_cb(const geometry_msgs::msg::Point::SharedPtr msg);

  trajectory_msgs::msg::MultiDOFJointTrajectory toMultiDOF(
      const vola::trajectory_t& traj);

  bool has_odom_ = false;

  Eigen::Vector3d current_pos_;

  MinJerkGenerator traj_generator_;

  double dt_{0.01};
  double default_duration_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cmd_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr
      traj_pub_;
};

PositionCommander::PositionCommander() : Node("position_commander") {
  default_duration_ = this->declare_parameter("trajectory_duration", 2.0);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 1,
      std::bind(&PositionCommander::odom_cb, this, std::placeholders::_1));

  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "command_pos", 1,
      std::bind(&PositionCommander::command_cb, this, std::placeholders::_1));

  traj_pub_ =
      this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
          "cmd_trajectory", 10);

  RCLCPP_INFO(this->get_logger(), "Position Commander initialized.");
}

void PositionCommander::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pos_ =
      Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
  has_odom_ = true;
}

void PositionCommander::command_cb(
    const geometry_msgs::msg::Point::SharedPtr msg) {
  if (!has_odom_) {
    return;
  }

  Eigen::Vector3d goal(msg->x, msg->y, msg->z);
  auto            traj = traj_generator_.get_trajectory(current_pos_, goal,
                                                        default_duration_, dt_);
  traj_pub_->publish(toMultiDOF(traj));
}

trajectory_msgs::msg::MultiDOFJointTrajectory PositionCommander::toMultiDOF(
    const vola::trajectory_t& traj) {
  trajectory_msgs::msg::MultiDOFJointTrajectory msg;
  msg.header.stamp    = this->now();
  msg.header.frame_id = "odom";

  vola::to_multidof_trajectory(traj, msg, [](auto& pt, double t) {
    pt.time_from_start = rclcpp::Duration::from_seconds(t);
  });

  return msg;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionCommander>());
  rclcpp::shutdown();
  return 0;
}
