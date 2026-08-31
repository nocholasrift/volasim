#include <minjerk_generator.h>
#include <trajectory_conversion.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

class PositionCommander {
 public:
  PositionCommander(ros::NodeHandle& nh);
  void spin();

 private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void commandCallback(
      const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg);

  trajectory_msgs::MultiDOFJointTrajectory toMultiDOF(
      const vola::trajectory_t& traj);

  bool has_odom_ = false;

  Eigen::Vector3d current_pos_;

  MinJerkGenerator traj_generator_;

  double dt_{0.01};

  ros::Subscriber odom_sub_;
  ros::Subscriber cmd_sub_;
  ros::Publisher  traj_pub_;
};

PositionCommander::PositionCommander(ros::NodeHandle& nh) {
  odom_sub_ =
      nh.subscribe("/odometry", 1, &PositionCommander::odomCallback, this);

  cmd_sub_ = nh.subscribe("/command_pos", 1,
                          &PositionCommander::commandCallback, this);

  traj_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "/cmd_trajectory", 10);
}

void PositionCommander::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  current_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z;

  has_odom_ = true;
}

void PositionCommander::commandCallback(
    const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg) {
  if (!has_odom_) {
    return;
  }

  if (msg->positions.size() != 3) {
    ROS_WARN("Ignoring /command_pos without 3 entries for pos.");
    return;
  }

  double duration = msg->time_from_start.toSec();

  if (duration <= 0) {
    ROS_WARN("Ignoring /command_pos with non-positive duration %.3f", duration);
    return;
  }

  Eigen::Vector3d goal(msg->positions[0], msg->positions[1], msg->positions[2]);

  auto traj = traj_generator_.get_trajectory(current_pos_, goal, duration, dt_);
  traj_pub_.publish(toMultiDOF(traj));
}

trajectory_msgs::MultiDOFJointTrajectory PositionCommander::toMultiDOF(
    const vola::trajectory_t& traj) {
  trajectory_msgs::MultiDOFJointTrajectory msg;
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "odom";

  vola::to_multidof_trajectory(traj, msg, [](auto& pt, double t) {
    pt.time_from_start = ros::Duration(t);
  });

  return msg;
}

void PositionCommander::spin() {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "position_commander");
  ros::NodeHandle nh;

  PositionCommander node(nh);
  node.spin();

  return 0;
}
