#include <minjerk_generator.h>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>

enum class Mode {
  IDLE,
  EXECUTING
};

class PositionCommander {
public:
  PositionCommander(ros::NodeHandle& nh);
  void spin();

private:
  struct Axis{
    enum Index : size_t {
      X = 0,
      Y = 1,
      Z = 2,
    };

    static constexpr size_t DIMS = 3;
  };

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void commandCallback(
      const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg);
  void timerCallback(const ros::TimerEvent&);

  vola::trajectory_t generateTrajectory(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& goal,
      double duration,
      double dt);

  bool has_odom_ = false;

  Eigen::Vector3d current_pos_;
  Eigen::Vector3d goal_pos_;

  MinJerkGenerator traj_generator;

  vola::trajectory_t active_traj_;
  size_t traj_index_ = 0;
  double dt_{0.01};

  Mode mode_ = Mode::IDLE;

  ros::Subscriber odom_sub_;
  ros::Subscriber cmd_sub_;
  ros::Publisher traj_pub_;
  ros::Timer timer_;
};

PositionCommander::PositionCommander(ros::NodeHandle& nh) {

  odom_sub_ = nh.subscribe("/odometry", 1,
      &PositionCommander::odomCallback, this);

  cmd_sub_ = nh.subscribe("/command_pos", 1,
      &PositionCommander::commandCallback, this);

  traj_pub_ = nh.advertise<trajectory_msgs::JointTrajectoryPoint>(
      "/cmd_full_state", 10);

  timer_ = nh.createTimer(
      ros::Duration(dt_),
      &PositionCommander::timerCallback,
      this);
}


void PositionCommander::odomCallback(
    const nav_msgs::Odometry::ConstPtr& msg) {

  current_pos_ << msg->pose.pose.position.x,
                  msg->pose.pose.position.y,
                  msg->pose.pose.position.z;

  has_odom_ = true;
}

void PositionCommander::commandCallback(
    const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg) {

  if (!has_odom_ || mode_ == Mode::EXECUTING){
    return;   // ignore if busy or no odom
  }

  if (msg->positions.size() != Axis::DIMS){
    ROS_WARN("Ignoring /command_pos without 3 entries for pos.");
    return;
  }

  double duration = msg->time_from_start.toSec();

  if (duration <= 0){
    ROS_WARN("Ignoring /command_pos with non-positive duration %.3f", duration);
    return;
  }

  goal_pos_ << msg->positions[Axis::X],
               msg->positions[Axis::Y],
               msg->positions[Axis::Z];


  active_traj_ = generateTrajectory(current_pos_, goal_pos_, duration, dt_);

  traj_index_ = 0;
  mode_ = Mode::EXECUTING;
}

vola::trajectory_t PositionCommander::generateTrajectory(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    double duration,
    double dt){

  return traj_generator.get_trajectory(start, goal, duration, dt);
}

void PositionCommander::timerCallback(
    const ros::TimerEvent&) {

  if (mode_ == Mode::IDLE) {
    return;
  }

  if(traj_index_ >= active_traj_.states.size()){
    mode_ = Mode::IDLE;
    return;
  }

  const auto& state = active_traj_.states[traj_index_];

  trajectory_msgs::JointTrajectoryPoint msg;

  msg.positions     = {state.pos[Axis::X], state.pos[Axis::Y], state.pos[Axis::Z]};
  msg.velocities    = {state.vel[Axis::X], state.vel[Axis::Y], state.vel[Axis::Z]};
  msg.accelerations = {state.acc[Axis::X], state.acc[Axis::Y], state.acc[Axis::Z]};
  msg.effort        = {state.jerk[Axis::X], state.jerk[Axis::Y], state.jerk[Axis::Z]};
  msg.time_from_start = ros::Duration(traj_index_ * dt_);

  traj_pub_.publish(msg);

  ++traj_index_;
}

void PositionCommander::spin() {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "position_commander");
  ros::NodeHandle nh;

  PositionCommander node(nh);
  node.spin();

  return 0;
}
