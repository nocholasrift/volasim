#include <volasim/comms/frames.h>
#include <volasim/comms/msgs/DepthCamera.pb.h>
#include <volasim/comms/msgs/DroneState.pb.h>
#include <volasim/comms/msgs/Odometry.pb.h>
#include <volasim/comms/msgs/Thrust.pb.h>
#include <volasim/comms/msgs/Transform.pb.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <zmq.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

enum class Action { kTakeoff, kLand, kFlying, kIdle };

// Host the sim publishes state on. Defaults to localhost for native runs; set
// VOLASIM_SIM_HOST=host.docker.internal when the bridge runs in a container.
static std::string sim_state_endpoint() {
  const char* host = std::getenv("VOLASIM_SIM_HOST");
  return "tcp://" + std::string(host ? host : "localhost") + ":5556";
}

// Preferred cloud endpoint: ipc, so a native same-host consumer pays no network
// cost. VOLASIM_CLOUD_ENDPOINT overrides it (e.g. to force tcp).
static std::string cloud_primary_endpoint() {
  const char* ep = std::getenv("VOLASIM_CLOUD_ENDPOINT");
  return ep ? ep : "ipc:///tmp/volasim_cloud";
}

// TCP endpoint to fall back to when ipc yields nothing — the only path that
// crosses a container/VM boundary (ipc sockets are host-kernel objects and do
// not survive a bind mount into the Docker Desktop VM). Empty when the primary
// is already tcp, since there is nothing better to fall back to.
static std::string cloud_fallback_endpoint(const std::string& primary) {
  if (primary.rfind("tcp://", 0) == 0) {
    return "";
  }
  const char* host = std::getenv("VOLASIM_SIM_HOST");
  return "tcp://" + std::string(host ? host : "localhost") + ":5559";
}

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
    (void)sock.recv(part);  // the rest of an atomic message is already buffered
    frames.push_back(std::move(part));
  }
  return frames;
}

// state, tf and tf_static share one socket, so messages are routed by the exact
// topic suffix. A prefix test would be wrong: "drone/<id>/tf" is a byte-prefix
// of "drone/<id>/tf_static", so a prefix on tf would also swallow tf_static.
static bool ends_with(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Pulls <id> out of a "drone/<id>/<stream>" topic; 0 if it cannot be parsed.
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
    // Cloud publishers are created lazily, one per sensor stream, in
    // publish_cloud() — the set of sensors is not known until frames arrive.

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

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
    zmq_subscriber_.connect(sim_state_endpoint());
    zmq_subscriber_.set(zmq::sockopt::subscribe, "");
    // state, tf and tf_static interleave on this socket, so a depth-1 queue would
    // drop one stream between ticks; leave room for a short burst per drone.
    zmq_subscriber_.set(zmq::sockopt::rcvhwm, 10);

    cloud_endpoint_          = cloud_primary_endpoint();
    cloud_fallback_endpoint_ = cloud_fallback_endpoint(cloud_endpoint_);
    zmq_cloud_sub_ = zmq::socket_t(zmq_context_, zmq::socket_type::sub);
    zmq_cloud_sub_.set(zmq::sockopt::subscribe, "");
    // A few slots rather than one: multiple sensors share this socket and may
    // capture on the same render frame, so their clouds can arrive together
    // between the 1 ms polls that drain one message each.
    zmq_cloud_sub_.set(zmq::sockopt::rcvhwm, 6);
    zmq_cloud_sub_.connect(cloud_endpoint_);
    cloud_fallback_deadline_ =
        std::chrono::steady_clock::now() + kCloudFallbackGrace;
    RCLCPP_INFO(this->get_logger(), "[VolasimROS2Wrapper] cloud on %s%s",
                cloud_endpoint_.c_str(),
                cloud_fallback_endpoint_.empty()
                    ? ""
                    : (" (falls back to " + cloud_fallback_endpoint_ + ")")
                          .c_str());

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
                std::shared_ptr<std_srvs::srv::Empty::Response>      response) {
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

    poll_fast();
    poll_cloud();
  }

  // Drains the fast socket and routes each message by its topic; state, tf and
  // tf_static share this socket. The batch is capped so a saturated fast stream
  // cannot starve poll_cloud(), which runs after this each tick — well above the
  // steady-state volume, so it only bounds a pathological burst.
  static constexpr int kMaxFastMsgsPerTick = 32;

  void poll_fast() {
    for (int i = 0; i < kMaxFastMsgsPerTick; ++i) {
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
      RCLCPP_WARN(this->get_logger(),
                  "[VolasimROS2Wrapper] Failed to parse tf message");
      return;
    }

    std::vector<geometry_msgs::msg::TransformStamped> out;
    out.reserve(tf_msg.transforms_size());
    for (const volasim_msgs::TransformStamped& t : tf_msg.transforms()) {
      geometry_msgs::msg::TransformStamped ts;
      ts.header.stamp            = rclcpp::Time(t.header().stamp_ns());
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
      static_tf_broadcaster_->sendTransform(out);
    } else {
      tf_broadcaster_->sendTransform(out);
    }
  }

  void handle_state(const std::vector<zmq::message_t>& frames) {
    const std::string topic(static_cast<const char*>(frames[0].data()),
                            frames[0].size());
    const uint32_t drone_id = drone_id_from_topic(topic);

    // Parse the DroneState wrapper and pull out odom rather than parsing the
    // bytes as a bare Odometry, whose fields do not line up.
    volasim_msgs::DroneState state;
    if (!state.ParseFromArray(frames[1].data(), frames[1].size())) {
      RCLCPP_WARN(this->get_logger(),
                  "[VolasimROS2Wrapper] Failed to parse drone state message");
      return;
    }
    const volasim_msgs::Odometry& odom = state.odom();

    nav_msgs::msg::Odometry msg;
    // Match the tf tree so the odom resolves against odom -> base_link.
    msg.header.stamp       = this->now();
    msg.header.frame_id    = volasim::frames::odom(drone_id);
    msg.child_frame_id     = volasim::frames::baseLink(drone_id);
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

  // Any frame at all proves the primary endpoint delivers; a malformed message
  // is still delivery, so it too cancels the fallback.
  void poll_cloud() {
    // Cloud is multipart: [topic] [DepthCamera header] [uint16 mm payload].
    std::vector<zmq::message_t> frames = recv_multipart(zmq_cloud_sub_);
    if (frames.empty()) {
      maybe_fallback_cloud();
      return;
    }
    cloud_received_ = true;
    if (frames.size() < 3) {
      return;
    }

    const std::string topic(static_cast<const char*>(frames[0].data()),
                            frames[0].size());

    volasim_msgs::DepthCamera header;
    if (!header.ParseFromArray(frames[1].data(), frames[1].size())) {
      RCLCPP_WARN(this->get_logger(),
                  "[VolasimROS2Wrapper] Failed to parse depth header");
      return;
    }
    publish_cloud(topic, header, frames[2]);
  }

  // Maps a sensor's ZMQ topic to its ROS point-cloud topic:
  //   drone/<id>/<sensor>/depth  ->  drone_<id>/<sensor>/points
  // so each sensor lands on its own PointCloud2 topic instead of alternating
  // with other sensors on one shared topic (which flickers in a viewer).
  //
  // The "drone/<id>" prefix is folded to "drone_<id>" to match the tf frame
  // namespace and, crucially, because a ROS topic name token may not start with
  // a digit: "drone/0/..." has a bare "0" token and is rejected, "drone_0" is
  // one valid token.
  static std::string ros_cloud_topic(const std::string& zmq_topic) {
    static constexpr std::string_view kDepthSuffix = "/depth";
    std::string                       base         = zmq_topic;
    if (base.size() >= kDepthSuffix.size() &&
        base.compare(base.size() - kDepthSuffix.size(), kDepthSuffix.size(),
                     kDepthSuffix) == 0) {
      base.resize(base.size() - kDepthSuffix.size());
    }

    // Fold the drone separator into the name so "drone/<id>" becomes one token.
    const std::size_t drone_sep = base.find('/');
    if (drone_sep != std::string::npos) {
      base[drone_sep] = '_';
    }

    return base + "/points";
  }

  // Returns the cloud publisher for a sensor topic, creating it on first sight.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_for(
      const std::string& zmq_topic) {
    auto it = cloud_pubs_.find(zmq_topic);
    if (it == cloud_pubs_.end()) {
      auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          ros_cloud_topic(zmq_topic), 10);
      it = cloud_pubs_.emplace(zmq_topic, std::move(pub)).first;
    }
    return it->second;
  }

  // ipc gives nothing when the sim is on the far side of a container/VM
  // boundary. After a grace period with no frames, switch the same socket over
  // to tcp — one-shot, since tcp reaches the sim whether it is local or remote.
  void maybe_fallback_cloud() {
    if (cloud_received_ || cloud_fell_back_ || cloud_fallback_endpoint_.empty()) {
      return;
    }
    if (std::chrono::steady_clock::now() < cloud_fallback_deadline_) {
      return;
    }
    RCLCPP_WARN(this->get_logger(),
                "[VolasimROS2Wrapper] no cloud on %s; falling back to %s",
                cloud_endpoint_.c_str(), cloud_fallback_endpoint_.c_str());
    zmq_cloud_sub_.disconnect(cloud_endpoint_);
    zmq_cloud_sub_.connect(cloud_fallback_endpoint_);
    cloud_endpoint_  = cloud_fallback_endpoint_;
    cloud_fell_back_ = true;
  }

  void publish_cloud(const std::string&               zmq_topic,
                     const volasim_msgs::DepthCamera& header,
                     const zmq::message_t&            payload) {
    const uint32_t w = header.width();
    const uint32_t h = header.height();
    if (payload.size() < static_cast<size_t>(w) * h * sizeof(uint16_t)) {
      RCLCPP_WARN(this->get_logger(),
                  "[VolasimROS2Wrapper] depth payload smaller than w*h");
      return;
    }

    const auto* depth = static_cast<const uint16_t*>(payload.data());
    const float fx = header.fx(), fy = header.fy();
    const float cx = header.cx(), cy = header.cy();

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = header.header().frame_id();
    cloud.header.stamp    = rclcpp::Time(header.header().stamp_ns());
    cloud.is_bigendian    = false;
    cloud.is_dense        = false;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(static_cast<size_t>(w) * h);  // upper bound; trimmed below

    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");

    size_t points = 0;
    for (uint32_t row = 0; row < h; ++row) {
      // glReadPixels gives row 0 at the bottom; flip to the top-origin optical
      // frame so the cloud is not upside down.
      const uint32_t v = h - 1 - row;
      for (uint32_t u = 0; u < w; ++u) {
        const uint16_t d = depth[row * w + u];
        if (d == 0) {  // 0 is the sensor's no-return marker
          continue;
        }
        const float z = static_cast<float>(d) * 0.001F;
        *ix           = (static_cast<float>(u) - cx) * z / fx;
        *iy           = (static_cast<float>(v) - cy) * z / fy;
        *iz           = z;
        ++ix, ++iy, ++iz, ++points;
      }
    }
    mod.resize(points);
    cloud_pub_for(zmq_topic)->publish(cloud);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pos_cmd_pub_;

  // One PointCloud2 publisher per sensor, keyed by the sensor's ZMQ topic.
  std::unordered_map<std::string,
                     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
      cloud_pubs_;

  std::unique_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr takeoff_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr land_srv_;

  std::array<float, 4> input_{0, 0, 0, 0};

  std::queue<Action> pending_actions_;
  Action             state_ = Action::kIdle;

  zmq::context_t zmq_context_;
  zmq::socket_t  zmq_subscriber_;
  zmq::socket_t  zmq_cloud_sub_;
  zmq::socket_t  zmq_publisher_;

  // Long enough that a slow-to-start sim is not mistaken for an unreachable ipc
  // endpoint, short enough that a container consumer is not starved of cloud.
  static constexpr std::chrono::seconds kCloudFallbackGrace{5};
  std::string                           cloud_endpoint_;
  std::string                           cloud_fallback_endpoint_;
  std::chrono::steady_clock::time_point cloud_fallback_deadline_;
  bool                                  cloud_received_  = false;
  bool                                  cloud_fell_back_ = false;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VolasimROS2Wrapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
