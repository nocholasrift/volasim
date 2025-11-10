#include "lee_controller_ros1.h"

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
  // params_["kR"] = 30.0;
  // params_["kR"] = 8.81;
  // params_["kw"] = 2.54;
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

void LeeControlNode::position_cb(const geometry_msgs::Point::ConstPtr& msg) {
  desired_state_.reset();
  desired_state_.pos = Eigen::Vector3d(msg->x, msg->y, msg->z);

  traj_ = generateTraj(state_, desired_state_, 6.);

  state_set_ = true;
  start_ = ros::Time::now();
}

void LeeControlNode::control_loop(const ros::TimerEvent&) {
  if (!initialized_ || !state_set_)
    return;

  if (traj_.points.size() == 0)
    return;

  double t = (ros::Time::now() - start_).toSec();
  int ind = 0;

  for (const trajectory_msgs::JointTrajectoryPoint& pt : traj_.points) {
    if (t < pt.time_from_start.toSec()) {
      break;
    }
    ind++;
  }
  ind = std::min(static_cast<int>(traj_.points.size() - 1), ind);

  trajectory_msgs::JointTrajectoryPoint pt = traj_.points[ind];

  vola::state_t desired_s;
  desired_s.pos =
      Eigen::Vector3d(pt.positions[0], pt.positions[1], pt.positions[2]);
  desired_s.vel =
      Eigen::Vector3d(pt.velocities[0], pt.velocities[1], pt.velocities[2]);

  desired_s.acc = Eigen::Vector3d(pt.accelerations[0], pt.accelerations[1],
                                  pt.accelerations[2]);

  desired_s.jerk = Eigen::Vector3d(pt.effort[0], pt.effort[1], pt.effort[2]);

  Eigen::Vector4d cmd = controller_.computeControls(state_, desired_s);

  std_msgs::Float32MultiArray msg;
  std_msgs::MultiArrayDimension dim;
  dim.size = 4;
  msg.layout.dim.push_back(dim);

  msg.data.push_back(cmd[0]);
  msg.data.push_back(cmd[1]);
  msg.data.push_back(cmd[2]);
  msg.data.push_back(cmd[3]);

  cmd_pub_.publish(msg);
}

trajectory_msgs::JointTrajectory LeeControlNode::generateTraj(
    const vola::state_t& start, const vola::state_t& end, double T) {
  // min jerk trajectory generation
  // ax5 + bx4 + cx3 + dx2 + ex + f

  Eigen::MatrixXd t_mat(6, 6);
  t_mat.row(0) << 0., 0., 0., 0., 0., 1.;
  t_mat.row(1) << T * T * T * T * T, T * T * T * T, T * T * T, T * T, T, 1;
  t_mat.row(2) << 0., 0., 0., 0., 1., 0.;
  t_mat.row(3) << 5 * T * T * T * T, 4 * T * T * T, 3 * T * T, 2 * T, 1., 0.;
  t_mat.row(4) << 0., 0., 0., 2., 0., 0.;
  t_mat.row(5) << 20 * T * T * T, 12 * T * T, 6 * T, 2, 0., 0.;

  Eigen::MatrixXd t_mat_inv = t_mat.inverse();

  Eigen::VectorXd bounds(6);
  std::array<Eigen::VectorXd, 3> coeffs;
  for (int i = 0; i < 3; ++i) {
    bounds << start.pos(i), end.pos(i), start.vel(i), end.vel(i), 0., 0.;
    coeffs[i] = t_mat_inv * bounds;
  }

  auto eval_poly = [this](const Eigen::VectorXd& c, double t) {
    double ret = 0;
    double t_pow = 1;
    for (int i = 0; i < c.size(); ++i) {
      ret += c[c.size() - 1 - i] * t_pow;
      t_pow *= t;
    }

    return ret;
  };

  // turn into joint trajectory
  trajectory_msgs::JointTrajectory msg;
  size_t sz = static_cast<size_t>(T / 0.05) + 1;
  double t = 0.;
  msg.points.reserve(sz);
  for (size_t i = 0; i < sz; ++i) {
    trajectory_msgs::JointTrajectoryPoint& pt = msg.points.emplace_back();

    for (int j = 0; j < 3; ++j) {
      pt.positions.push_back(eval_poly(coeffs[j], t));

      Eigen::VectorXd v_coeffs(5);
      v_coeffs << 5 * coeffs[j][0], 4 * coeffs[j][1], 3 * coeffs[j][2],
          2 * coeffs[j][3], coeffs[j][4];

      pt.velocities.push_back(eval_poly(v_coeffs, t));

      Eigen::VectorXd a_coeffs(4);
      a_coeffs << 20 * coeffs[j][0], 12 * coeffs[j][1], 6 * coeffs[j][2],
          2 * coeffs[j][3];

      pt.accelerations.push_back(eval_poly(a_coeffs, t));

      Eigen::VectorXd j_coeffs(3);
      j_coeffs << 60 * coeffs[j][0], 24 * coeffs[j][1], 6 * coeffs[j][2];

      pt.effort.push_back(eval_poly(j_coeffs, t));
    }
    // std::cout << pt.effort[0] << " " << pt.effort[1] << " " << pt.effort[2]
    //           << "\n";
    // std::cout << pt.accelerations[0] << " " << pt.accelerations[1] << " "
    //           << pt.accelerations[2] << "\n";

    pt.time_from_start = ros::Duration(t);

    t += 0.05;
  }

  return msg;
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
