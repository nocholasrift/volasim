#include "lee_controller_ros1.h"
#include "trajectory_conversion.h"

LeeControlNode::LeeControlNode(ros::NodeHandle& nh) {
  odom_sub_ = nh.subscribe("odometry", 1, &LeeControlNode::odom_cb, this);
  traj_sub_ =
      nh.subscribe("cmd_trajectory", 1, &LeeControlNode::trajectory_cb, this);

  cmd_pub_ = nh.advertise<std_msgs::Float32MultiArray>("command", 10);

  control_loop_timer_ =
      nh.createTimer(ros::Duration(.005), &LeeControlNode::control_loop, this);

  initialized_ = false;
  traj_set_    = false;

  params_["kp"]       = 69.44;
  params_["kv"]       = 24.304;
  params_["kR"]       = 13.81;
  params_["kw"]       = 2.54;
  params_["mass"]     = 4.34;
  params_["length"]   = 0.315;
  params_["c_torque"] = 8.004e-4;
  params_["j0"]       = 0.0820;
  params_["j1"]       = 0.0845;
  params_["j2"]       = 0.1377;

  controller_.loadParams(params_);

  conv_mat_ << 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1;

  double l          = params_["length"];
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

  Eigen::Quaterniond quat(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

  state_.rot = quat.toRotationMatrix();

  initialized_ = true;
}

void LeeControlNode::trajectory_cb(
    const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
  bool ok = vola::from_multidof_trajectory(
      *msg, active_traj_,
      [](const auto& pt) { return pt.time_from_start.toSec(); });

  if (!ok) {
    ROS_WARN("Rejected trajectory: empty or non-monotonic timestamps");
    return;
  }

  start_    = ros::Time::now();
  traj_set_ = true;
}

void LeeControlNode::control_loop(const ros::TimerEvent&) {
  if (!initialized_ || !traj_set_) {
    return;
  }

  double          t         = (ros::Time::now() - start_).toSec();
  const auto&     desired_s = active_traj_.at_time(t);
  Eigen::Vector4d cmd       = controller_.computeControls(state_, desired_s);

  std_msgs::Float32MultiArray msg;
  msg.data = {
      static_cast<float>(cmd[Motors::M1]), static_cast<float>(cmd[Motors::M2]),
      static_cast<float>(cmd[Motors::M3]), static_cast<float>(cmd[Motors::M4])};

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
