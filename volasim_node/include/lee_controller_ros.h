#ifndef VOLASIM_NODE_LEE_CONTROLLER_ROS_H
#define VOLASIM_NODE_LEE_CONTROLLER_ROS_H

#include "lee_controller.h"

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <Eigen/Core>

#include <string_view>
#include <unordered_map>

class LeeControlNode {
 public:
  LeeControlNode(ros::NodeHandle& nh);

  void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);

  void position_cb(const geometry_msgs::Point::ConstPtr& msg);

  void spin();

 private:
  void control_loop(const ros::TimerEvent&);

  ros::Timer control_loop_timer_;

  ros::Subscriber odom_sub_;
  ros::Subscriber desired_pos_sub_;

  ros::Publisher cmd_pub_;

  std::unordered_map<std::string_view, double> params_;

  bool initialized_;
  bool state_set_;

  vola::state_t state_;
  vola::state_t desired_state_;

  vola::LeeController controller_;
};

#endif
