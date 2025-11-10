#include <volasim/comms/msgs/Odometry.pb.h>
#include <volasim/comms/msgs/Thrust.pb.h>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <zmq.hpp>

#include <array>
#include <iostream>
#include <queue>

enum class Action { kTakeoff, kLand, kFlying, kIdle };

class VolasimROS2Wrapper : public rclcpp::Node {
 public:
  VolasimROS2Wrapper() : Node("volasim_ros2") {
    // Subscriber
    cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/command", 10,
        std::bind(&VolasimROS2Wrapper::cmd_cb, this, std::placeholders::_1));

    // Publisher
    state_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    pos_cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Point>("command_pos", 10);

    // Timer: 1 ms
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(1),
                                std::bind(&VolasimROS2Wrapper::timer_cb, this));

    // Service
    takeoff_srv_ = this->create_service<std_srvs::srv::Empty>(
        "takeoff", std::bind(&VolasimROS2Wrapper::takeoff_srv, this,
                             std::placeholders::_1, std::placeholders::_2));

    land_srv_ = this->create_service<std_srvs::srv::Empty>(
        "land", std::bind(&VolasimROS2Wrapper::land_srv, this,
                          std::placeholders::_1, std::placeholders::_2));

    // ZMQ setup
    zmq_subscriber_ = zmq::socket_t(zmq_context_, zmq::socket_type::sub);
    zmq_subscriber_.connect("tcp://localhost:5556");
    zmq_subscriber_.set(zmq::sockopt::subscribe, "");
    zmq_subscriber_.set(zmq::sockopt::rcvhwm, 1);

    zmq_publisher_ = zmq::socket_t(zmq_context_, zmq::socket_type::pub);
    zmq_publisher_.bind("tcp://*:5557");
  }

 private:
  void cmd_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 4) {
      RCLCPP_WARN(this->get_logger(),
                  "[VolasimROS2Wrapper] input vector does not contain 4 "
                  "values!");
      return;
    }
    input_ = {msg->data[0], msg->data[1], msg->data[2], msg->data[3]};
  }

  void takeoff_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    pending_actions_.push(Action::kTakeoff);
  }

  void land_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    pending_actions_.push(Action::kLand);
  }

  void publish_cmd() {
    volasim_msgs::Thrust thrust_msg;
    thrust_msg.set_f1(input_[0]);
    thrust_msg.set_f2(input_[1]);
    thrust_msg.set_f3(input_[2]);
    thrust_msg.set_f4(input_[3]);

    std::string data;
    if (!thrust_msg.SerializeToString(&data)) {
      RCLCPP_WARN(this->get_logger(),
                  "[VolasimROS2Wrapper] Failed to serialize Thrust message.");
      return;
    }

    zmq::message_t message(data.size());
    memcpy(message.data(), data.data(), data.size());
    zmq_publisher_.send(message, zmq::send_flags::none);
  }

  void timer_cb() {
    // If no command publisher, reset input
    if (cmd_sub_->get_publisher_count() == 0) {
      input_ = {0, 0, 0, 0};
    }

    volasim_msgs::Odometry odom;

    zmq::message_t update;
    auto result = zmq_subscriber_.recv(update, zmq::recv_flags::dontwait);
    if (!result.has_value())
      return;

    if (!odom.ParseFromArray(update.data(), update.size())) {
      RCLCPP_WARN(this->get_logger(),
                  "[VolasimROS2Wrapper] Failed to parse odometry message");
      return;
    }

    nav_msgs::msg::Odometry msg;
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

    state_pub_->publish(msg);

    publish_cmd();

    // parse any pending actions
    if (pending_actions_.size() > 0) {
      switch (pending_actions_.front()) {
        case Action::kTakeoff:
          if (state_ == Action::kIdle) {
            geometry_msgs::msg::Point p;
            p.x = odom.position().x();
            p.y = odom.position().y();
            p.z = 1.0;
            pos_cmd_pub_->publish(p);
            state_ = Action::kTakeoff;
            pending_actions_.pop();
          }
          break;
        case Action::kLand:
          if (state_ == Action::kFlying) {
            geometry_msgs::msg::Point p;
            p.x = odom.position().x();
            p.y = odom.position().y();
            p.z = 0.01;
            pos_cmd_pub_->publish(p);
            state_ = Action::kLand;
            pending_actions_.pop();
          }
          break;
      }
    }

    if (state_ == Action::kTakeoff && odom.position().z() > 1.9)
      state_ = Action::kFlying;

    if (state_ == Action::kLand && odom.position().z() < 0.02)
      state_ = Action::kIdle;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pos_cmd_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr takeoff_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr land_srv_;

  std::array<float, 4> input_{0, 0, 0, 0};

  std::queue<Action> pending_actions_;
  Action state_ = Action::kIdle;

  zmq::context_t zmq_context_;
  zmq::socket_t zmq_subscriber_;
  zmq::socket_t zmq_publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VolasimROS2Wrapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
