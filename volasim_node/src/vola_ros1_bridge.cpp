/*#include <volasim/comms/msgs/Odometry.pb.h>*/
#include <volasim/comms/frames.h>
#include <volasim/comms/msgs/DroneState.pb.h>
#include <volasim/comms/msgs/Thrust.pb.h>
#include <volasim/comms/msgs/Transform.pb.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_srvs/Empty.h>
#include <zmq.hpp>

#include <array>
#include <cstdint>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

enum class Action { kTakeoff, kLand, kFlying, kIdle };

// Reads one whole multipart message, or returns empty when nothing is queued.
static std::vector<zmq::message_t> recv_multipart(zmq::socket_t& sock) {
  std::vector<zmq::message_t> frames;

  zmq::message_t first;
  if (!sock.recv(first, zmq::recv_flags::dontwait).has_value()) {
    return frames;
  }
  frames.push_back(std::move(first));

  while (sock.get(zmq::sockopt::rcvmore)) {
    zmq::message_t part;
    (void)sock.recv(part);
    frames.push_back(std::move(part));
  }
  return frames;
}

// Route by exact topic suffix, never a prefix: "drone/<id>/tf" is a byte-prefix
// of "drone/<id>/tf_static", so a prefix test on tf would also match tf_static.
static bool ends_with(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

static uint32_t drone_id_from_topic(const std::string& topic) {
  const size_t first = topic.find('/');
  if (first == std::string::npos) {
    return 0;
  }
  const size_t      second = topic.find('/', first + 1);
  const std::string id     = topic.substr(first + 1, second - first - 1);
  try {
    return static_cast<uint32_t>(std::stoul(id));
  } catch (const std::exception&) {
    return 0;
  }
}

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
    // state, tf and tf_static interleave on this socket, so a depth-1 queue would
    // drop one stream between ticks; leave room for a short burst per drone.
    zmq_subscriber_.set(zmq::sockopt::rcvhwm, 10);

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

    // Drain every queued message; state, tf and tf_static share this socket.
    for (;;) {
      std::vector<zmq::message_t> frames = recv_multipart(zmq_subscriber_);
      if (frames.size() < 2) {
        return;
      }
      const std::string topic(static_cast<const char*>(frames[0].data()),
                              frames[0].size());
      if (ends_with(topic, "/state")) {
        handle_state(frames);
      } else if (ends_with(topic, "/tf_static")) {
        handle_tf(frames, /*is_static=*/true);
      } else if (ends_with(topic, "/tf")) {
        handle_tf(frames, /*is_static=*/false);
      }
    }
  }

  // Maps one TFMessage onto a tf2 broadcast. Frame names come straight off the
  // wire — the sim owns the tf tree convention, the bridge just forwards it.
  void handle_tf(const std::vector<zmq::message_t>& frames, bool is_static) {
    volasim_msgs::TFMessage tf_msg;
    if (!tf_msg.ParseFromArray(frames[1].data(), frames[1].size())) {
      ROS_WARN("[VolasimROSWrapper] Failed to parse tf message");
      return;
    }

    std::vector<geometry_msgs::TransformStamped> out;
    out.reserve(tf_msg.transforms_size());
    for (const volasim_msgs::TransformStamped& t : tf_msg.transforms()) {
      geometry_msgs::TransformStamped ts;
      ts.header.stamp.fromNSec(t.header().stamp_ns());
      ts.header.frame_id         = t.header().frame_id();
      ts.child_frame_id          = t.child_frame_id();
      ts.transform.translation.x = t.translation().x();
      ts.transform.translation.y = t.translation().y();
      ts.transform.translation.z = t.translation().z();
      ts.transform.rotation.x    = t.rotation().x();
      ts.transform.rotation.y    = t.rotation().y();
      ts.transform.rotation.z    = t.rotation().z();
      ts.transform.rotation.w    = t.rotation().w();
      out.push_back(ts);
    }

    if (out.empty()) {
      return;
    }
    if (is_static) {
      static_tf_broadcaster_.sendTransform(out);
    } else {
      tf_broadcaster_.sendTransform(out);
    }
  }

  void handle_state(const std::vector<zmq::message_t>& frames) {
    const std::string topic(static_cast<const char*>(frames[0].data()),
                            frames[0].size());
    const uint32_t drone_id = drone_id_from_topic(topic);

    volasim_msgs::DroneState drone_state;
    if (!drone_state.ParseFromArray(frames[1].data(), frames[1].size())) {
      ROS_WARN("[VolasimROSWrapper] Failed to parse odometry message");
      return;
    }

    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    // Match the tf tree so the odom resolves against odom -> base_link.
    msg.header.frame_id = volasim::frames::odom(drone_id);
    msg.child_frame_id = volasim::frames::baseLink(drone_id);
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
    imu_msg.header.frame_id = volasim::frames::baseLink(drone_id);
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

  tf2_ros::TransformBroadcaster       tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

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
