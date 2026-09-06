#ifndef VOLASIM_NODE_LEE_CONTROLLER_ROS1_H
#define VOLASIM_NODE_LEE_CONTROLLER_ROS1_H

#include "lee_controller.h"

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <Eigen/Core>

#include <string_view>
#include <unordered_map>

class LeeControlNode {
 public:
  LeeControlNode(ros::NodeHandle& nh);

  void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
  void trajectory_cb(
      const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);

  void spin();

 private:
  struct Motors {
    enum Index : size_t { M1 = 0, M2 = 1, M3 = 2, M4 = 3 };
    static constexpr size_t N_MOTORS = 4;
  };

  void control_loop(const ros::TimerEvent&);

  ros::Timer control_loop_timer_;

  ros::Subscriber odom_sub_;
  ros::Subscriber traj_sub_;

  ros::Publisher cmd_pub_;

  ros::Time start_;

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
