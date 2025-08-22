#include <volasim/comms/msgs/Odometry.pb.h>
#include <volasim/comms/msgs/Thrust.pb.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <zmq.hpp>

#include <iostream>
#include <sstream>

class VolasimROSWrapper {
 public:
  VolasimROSWrapper(ros::NodeHandle& nh) {
    cmd_sub_ = nh.subscribe("/command", 1, &VolasimROSWrapper::cmd_cb, this);

    state_pub_ = nh.advertise<nav_msgs::Odometry>("odometry", 10);

    timer_ =
        nh.createTimer(ros::Duration(.001), &VolasimROSWrapper::timer_cb, this);

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

  void publish_cmd() {
    volasim_msgs::Thrust thrust_msg;
    thrust_msg.set_f1(input_[0]);
    thrust_msg.set_f2(input_[1]);
    thrust_msg.set_f3(input_[2]);
    thrust_msg.set_f4(input_[3]);

    std::string data;
    thrust_msg.SerializeToString(&data);

    zmq::message_t message(data.size());
    memcpy(message.data(), data.data(), data.size());
    zmq_publisher_.send(message, zmq::send_flags::none);
  }

  void timer_cb(const ros::TimerEvent&) {

    if (cmd_sub_.getNumPublishers() == 0) {
      input_ = {0, 0, 0, 0};
    }

    volasim_msgs::Odometry odom;

    zmq::message_t update;
    auto result = zmq_subscriber_.recv(update, zmq::recv_flags::dontwait);

    if (!result.has_value()) {
      return;
    }

    if (!odom.ParseFromArray(update.data(), update.size())) {
      ROS_WARN("[VolasimROSWrapper] Failed to parse odometry message");
    }

    nav_msgs::Odometry msg;
    msg.pose.pose.position.x = odom.position().x();
    msg.pose.pose.position.y = odom.position().y();
    msg.pose.pose.position.z = odom.position().z();

    msg.pose.pose.orientation.x = odom.orientation().x();
    msg.pose.pose.orientation.y = odom.orientation().y();
    msg.pose.pose.orientation.z = odom.orientation().z();
    msg.pose.pose.orientation.w = odom.orientation().w();

    msg.twist.twist.linear.x = odom.linvel().x();
    msg.twist.twist.linear.y = odom.linvel().y();
    msg.twist.twist.linear.z = odom.linvel().z();

    msg.twist.twist.angular.x = odom.angvel().x();
    msg.twist.twist.angular.y = odom.angvel().y();
    msg.twist.twist.angular.z = odom.angvel().z();

    state_pub_.publish(msg);

    publish_cmd();
  }

 private:
  ros::Publisher state_pub_;
  ros::Subscriber cmd_sub_;

  ros::Timer timer_;

  std::array<float, 4> input_{0, 0, 0, 0};

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
