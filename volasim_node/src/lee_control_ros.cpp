#include "lee_controller_ros.h"

#include <std_msgs/Float32MultiArray.h>

LeeControlNode::LeeControlNode(ros::NodeHandle& nh) {
  odom_sub_ = nh.subscribe("odometry", 1, &LeeControlNode::odom_cb, this);

  desired_pos_sub_ =
      nh.subscribe("command_pos", 1, &LeeControlNode::position_cb, this);

  cmd_pub_ = nh.advertise<std_msgs::Float32MultiArray>("command", 10);

  control_loop_timer_ =
      nh.createTimer(ros::Duration(.01), &LeeControlNode::control_loop, this);

  initialized_ = false;
  state_set_ = false;

  params_["kp"] = 69.44;
  params_["kv"] = 24.304;
  params_["kR"] = 30.0;
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

void LeeControlNode::position_cb(const geometry_msgs::Point::ConstPtr& msg) {
  desired_state_.reset();
  desired_state_.pos = Eigen::Vector3d(msg->x, msg->y, msg->z);

  state_set_ = true;
}

void LeeControlNode::control_loop(const ros::TimerEvent&) {
  if (!initialized_ || !state_set_)
    return;

  Eigen::Vector4d cmd = controller_.computeControls(state_, desired_state_);

  Eigen::Vector4d forces = conv_mat_inverse_ * cmd;

  std_msgs::Float32MultiArray msg;
  std_msgs::MultiArrayDimension dim;
  dim.size = 4;
  msg.layout.dim.push_back(dim);

  msg.data.push_back(forces[0]);
  msg.data.push_back(forces[1]);
  msg.data.push_back(forces[2]);
  msg.data.push_back(forces[3]);

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
