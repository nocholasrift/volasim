#include "lee_controller_ros1.h"
#include "ros/console.h"

#include <std_msgs/Float32MultiArray.h>

LeeControlNode::LeeControlNode(ros::NodeHandle& nh) {
  odom_sub_ = nh.subscribe("odometry", 1, &LeeControlNode::odom_cb, this);
  full_state_cmd_sub_ = nh.subscribe("cmd_full_state", 1, &LeeControlNode::full_state_cb, this);

  cmd_pub_ = nh.advertise<std_msgs::Float32MultiArray>("command", 10);

  control_loop_timer_ =
      nh.createTimer(ros::Duration(.005), &LeeControlNode::control_loop, this);

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

  // Mixer matrix for X-configuration quadrotor:
  //   conv * [f1, f2, f3, f4]^T = [total_thrust, torque_x, torque_y, torque_z]^T
  //   forces = conv.inverse() * cmd
  //
  // Motor numbering (vehicle frame: X forward, Y left):
  //    1 (front-right)    2 (front-left)
  //           X axis
  //    3 (back-left)      4 (back-right)

  conv_mat_ << 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1;

  double l = params_["length"];
  conv_mat_.row(1) *= l / sqrt(2);
  conv_mat_.row(2) *= l / sqrt(2);
  conv_mat_.row(3) *= params_["c_torque"];

  conv_mat_inverse_ = conv_mat_.inverse();
}

void LeeControlNode::odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {

  state_.pos =
      Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
  state_.vel =
      Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                      msg->twist.twist.linear.z);

  state_.w =
      Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                      msg->twist.twist.angular.z);

  // get rotation matrix from quaternion
  Eigen::Quaterniond quat(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

  state_.rot = quat.toRotationMatrix();

  initialized_ = true;
}

void LeeControlNode::full_state_cb(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg){
  // ensure message has proper axis number, otherwise we drop it
  if (msg->positions.size() != Axis::DIMS || msg->velocities.size() != 3 ||
      msg->accelerations.size() != Axis::DIMS || msg->effort.size() != 3){
    ROS_WARN_THROTTLE(1.0, "[LeeControlNode] cmd_full_state must contain %lu position," 
                      "velocity, acceleration, and jerk values.", Axis::DIMS);
  }

  full_state_cmd_ = *msg;
  state_set_ = true;
}

void LeeControlNode::control_loop(const ros::TimerEvent&) {
  if (!initialized_ || !state_set_)
    return;

  double t = (ros::Time::now() - start_).toSec();
  int ind = 0;

  const auto& pt = full_state_cmd_;

  vola::state_t desired_s;
  desired_s.pos =
      Eigen::Vector3d(pt.positions[Axis::X], pt.positions[Axis::Y], pt.positions[Axis::Z]);
  desired_s.vel =
      Eigen::Vector3d(pt.velocities[Axis::X], pt.velocities[Axis::Y], pt.velocities[Axis::Z]);

  desired_s.acc = Eigen::Vector3d(pt.accelerations[Axis::X], pt.accelerations[Axis::Y],
                                  pt.accelerations[Axis::Z]);

  desired_s.jerk = Eigen::Vector3d(pt.effort[Axis::X], pt.effort[Axis::Y], pt.effort[Axis::Z]);

  Eigen::Vector4d cmd = controller_.computeControls(state_, desired_s);

  std_msgs::Float32MultiArray msg;
  std_msgs::MultiArrayDimension dim;
  dim.size = Motors::N_MOTORS;
  msg.layout.dim.push_back(dim);

  msg.data.push_back(cmd[Motors::M1]);
  msg.data.push_back(cmd[Motors::M2]);
  msg.data.push_back(cmd[Motors::M3]);
  msg.data.push_back(cmd[Motors::M4]);

  cmd_pub_.publish(msg);
}

void LeeControlNode::spin() {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "lee_controller");
  ros::NodeHandle nh;

  LeeControlNode node(nh);
  node.spin();
  return 0;
}
