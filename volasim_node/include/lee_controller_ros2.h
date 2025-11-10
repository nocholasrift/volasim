#ifndef VOLASIM_NODE_LEE_CONTROLLER_ROS_H
#define VOLASIM_NODE_LEE_CONTROLLER_ROS_H

#include "lee_controller.h"

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <Eigen/Core>

#include <string_view>
#include <unordered_map>

class LeeControlNode : public rclcpp::Node {
 public:
  LeeControlNode();

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);

  void position_cb(const geometry_msgs::msg::Point::SharedPtr msg);

  void spin();

 private:
  void control_loop();

  trajectory_msgs::msg::JointTrajectory generateTraj(const vola::state_t& start,
                                                     const vola::state_t& end,
                                                     double T);

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr desired_pos_sub_;

  rclcpp::Time start_;

  trajectory_msgs::msg::JointTrajectory traj_;

  std::unordered_map<std::string_view, double> params_;

  bool initialized_;
  bool state_set_;

  vola::state_t state_;
  vola::state_t desired_state_;

  vola::LeeController controller_;

  Eigen::Matrix4d conv_mat_;
  Eigen::Matrix4d conv_mat_inverse_;
};

#endif
