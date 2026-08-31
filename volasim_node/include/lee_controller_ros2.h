#ifndef VOLASIM_NODE_LEE_CONTROLLER_ROS2_H
#define VOLASIM_NODE_LEE_CONTROLLER_ROS2_H

#include "lee_controller.h"

#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <Eigen/Core>

#include <string_view>
#include <unordered_map>

class LeeControlNode : public rclcpp::Node {
 public:
  LeeControlNode();

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void trajectory_cb(
      const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg);

 private:
  struct Motors {
    enum Index : size_t { M1 = 0, M2 = 1, M3 = 2, M4 = 3 };
    static constexpr size_t N_MOTORS = 4;
  };

  void control_loop();

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr
      traj_sub_;

  rclcpp::Time start_;

  vola::trajectory_t active_traj_;

  std::unordered_map<std::string_view, double> params_;

  bool initialized_;
  bool traj_set_;

  vola::state_t state_;

  vola::LeeController controller_;

  Eigen::Matrix4d conv_mat_;
  Eigen::Matrix4d conv_mat_inverse_;
};

#endif
