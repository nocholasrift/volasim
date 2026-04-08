#ifndef VOLASIM_NODE_LEE_CONTROLLER_ROS_H
#define VOLASIM_NODE_LEE_CONTROLLER_ROS_H

#include "lee_controller.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <ros/ros.h>
#include <Eigen/Core>

#include <string_view>
#include <unordered_map>

class LeeControlNode {
 public:
  LeeControlNode(ros::NodeHandle& nh);

  void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
  void full_state_cb(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg);

  void spin();

 private:
  struct Axis {
    enum Index : size_t {
      X = 0, 
      Y = 1,
      Z = 2
    };

    static constexpr size_t DIMS = 3;
  };

  // I didn't like needing to static cast when making Motors an enum class
  // so I am doing it this way instead :)
  struct Motors{
    enum Index : size_t {
      M1 = 0, 
      M2 = 1,
      M3 = 2,
      M4 = 3
    };

    static constexpr size_t N_MOTORS = 4;
  };

  void control_loop(const ros::TimerEvent&);

  ros::Timer control_loop_timer_;

  ros::Subscriber odom_sub_;
  ros::Subscriber full_state_cmd_sub_;

  ros::Publisher cmd_pub_;

  ros::Time start_;

  trajectory_msgs::JointTrajectoryPoint full_state_cmd_;

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
