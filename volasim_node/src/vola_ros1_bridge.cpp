/*#include <volasim/comms/msgs/Odometry.pb.h>*/
#include <volasim/comms/msgs/DroneState.pb.h>
#include <volasim/comms/msgs/Thrust.pb.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_srvs/Empty.h>
#include <zmq.hpp>

#include <array>
#include <iostream>
#include <queue>

enum class Action { kTakeoff, kLand, kFlying, kIdle };

class VolasimROSWrapper {
 public:
  VolasimROSWrapper(ros::NodeHandle& nh) {
    cmd_sub_ = nh.subscribe("/command", 1, &VolasimROSWrapper::cmd_cb, this);

    state_pub_ = nh.advertise<nav_msgs::Odometry>("odometry", 10);
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu", 10);

    pos_cmd_pub_ = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("command_pos", 10);

    timer_ =
        nh.createTimer(ros::Duration(.001), &VolasimROSWrapper::timer_cb, this);

    // service
    takeoff_srv_ = nh.advertiseService("takeoff", &VolasimROSWrapper::takeoff_srv, this);
    land_srv_ = nh.advertiseService("land", &VolasimROSWrapper::land_srv, this);

    zmq_context_ = zmq::context_t(1);
    //  Socket to talk to server
    zmq_subscriber_ = zmq::socket_t(zmq_context_, zmq::socket_type::sub);
    zmq_subscriber_.connect("tcp://localhost:5556");
    zmq_subscriber_.set(zmq::sockopt::subscribe, "");
    zmq_subscriber_.set(zmq::sockopt::rcvhwm, 1);

    zmq_publisher_ = zmq::socket_t(zmq_context_, zmq::socket_type::pub);
    zmq_publisher_.bind("tcp://*:5557");
  }

  ~VolasimROSWrapper() {}

  void spin() {
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();
  }

  void cmd_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 4) {
      ROS_WARN(
          "[VolasimROSWrapper] input vector does not contain exacty 4 values!");
      return;
    }

    input_ = {msg->data[0], msg->data[1], msg->data[2], msg->data[3]};
  }

  bool takeoff_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    pending_actions_.push(Action::kTakeoff);

    return true;
  }

  bool land_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    pending_actions_.push(Action::kLand);

    return true;
  }

  void publish_cmd() {
    volasim_msgs::Thrust thrust_msg;
    thrust_msg.set_f1(input_[0]);
    thrust_msg.set_f2(input_[1]);
    thrust_msg.set_f3(input_[2]);
    thrust_msg.set_f4(input_[3]);

    std::string data;
    if (!thrust_msg.SerializeToString(&data)) {
      ROS_WARN("[VolasimROSWrapper] Failed to serialize Thrust message.");
      return;
    }

    zmq::message_t message(data.size());
    memcpy(message.data(), data.data(), data.size());
    zmq_publisher_.send(message, zmq::send_flags::none);
  }

  void timer_cb(const ros::TimerEvent&) {

    if (cmd_sub_.getNumPublishers() == 0) {
      input_ = {0, 0, 0, 0};
    }

    volasim_msgs::DroneState drone_state;

    zmq::message_t update;
    auto result = zmq_subscriber_.recv(update, zmq::recv_flags::dontwait);

    if (!result.has_value()) {
      return;
    }

    /*volasim_msgs::Odometry& odom = *drone_state.mutable_odom();*/
    /*volasim_msgs::Imu& imu = *drone_state.mutable_imu();*/
    if (!drone_state.ParseFromArray(update.data(), update.size())) {
      ROS_WARN("[VolasimROSWrapper] Failed to parse odometry message");
      return;
    }

    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = drone_state.odom().position().x();
    msg.pose.pose.position.y = drone_state.odom().position().y();
    msg.pose.pose.position.z = drone_state.odom().position().z();

    msg.pose.pose.orientation.x = drone_state.odom().orientation().x();
    msg.pose.pose.orientation.y = drone_state.odom().orientation().y();
    msg.pose.pose.orientation.z = drone_state.odom().orientation().z();
    msg.pose.pose.orientation.w = drone_state.odom().orientation().w();

    msg.twist.twist.linear.x = drone_state.odom().linvel().x();
    msg.twist.twist.linear.y = drone_state.odom().linvel().y();
    msg.twist.twist.linear.z = drone_state.odom().linvel().z();

    msg.twist.twist.angular.x = drone_state.odom().angvel().x();
    msg.twist.twist.angular.y = drone_state.odom().angvel().y();
    msg.twist.twist.angular.z = drone_state.odom().angvel().z();

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "base_link";
    imu_msg.orientation.x = drone_state.imu().orientation().x();
    imu_msg.orientation.y = drone_state.imu().orientation().y();
    imu_msg.orientation.z = drone_state.imu().orientation().z();
    imu_msg.orientation.w = drone_state.imu().orientation().w();

    imu_msg.angular_velocity.x = drone_state.imu().angvel().x();
    imu_msg.angular_velocity.y = drone_state.imu().angvel().y();
    imu_msg.angular_velocity.z = drone_state.imu().angvel().z();

    imu_msg.linear_acceleration.x = drone_state.imu().linacc().x();
    imu_msg.linear_acceleration.y = drone_state.imu().linacc().y();
    imu_msg.linear_acceleration.z = drone_state.imu().linacc().z();

    state_pub_.publish(msg);
    imu_pub_.publish(imu_msg);

    publish_cmd();
    
    // parse any pending actions
    if (pending_actions_.size() > 0) {
      switch (pending_actions_.front()) {
        case Action::kTakeoff:
          if (state_ == Action::kIdle) {
            trajectory_msgs::JointTrajectoryPoint pt;
            pt.positions = {drone_state.odom().position().x(), drone_state.odom().position().y(), 1.0};
            pt.time_from_start = ros::Duration(5.0);
            pos_cmd_pub_.publish(pt);
            state_ = Action::kTakeoff;
            pending_actions_.pop();
          }
          break;
        case Action::kLand:
          if (state_ == Action::kFlying) {
            trajectory_msgs::JointTrajectoryPoint pt;
            pt.positions = {drone_state.odom().position().x(), drone_state.odom().position().y(), 0.01};
            pt.time_from_start = ros::Duration(5.0);
            pos_cmd_pub_.publish(pt);
            state_ = Action::kLand;
            pending_actions_.pop();
          }
          break;
        default:
          std::cerr << "Action not implemented\n";
          break;
      }
    }

    if (state_ == Action::kTakeoff && drone_state.odom().position().z() > 0.95)
      state_ = Action::kFlying;

    if (state_ == Action::kLand && drone_state.odom().position().z() < 0.02)
      state_ = Action::kIdle;

  }

 private:
  ros::Publisher pos_cmd_pub_;
  ros::Publisher state_pub_;
  ros::Publisher imu_pub_;
  ros::Subscriber cmd_sub_;

  ros::ServiceServer takeoff_srv_;
  ros::ServiceServer land_srv_;

  ros::Timer timer_;

  std::array<float, 4> input_{0, 0, 0, 0};

  std::queue<Action> pending_actions_;
  Action state_ = Action::kIdle;

  zmq::context_t zmq_context_;
  zmq::socket_t zmq_subscriber_;
  zmq::socket_t zmq_publisher_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "volasim_ros");
  ros::NodeHandle nh;
  VolasimROSWrapper vola(nh);
  vola.spin();
  return 0;
}
