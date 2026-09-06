#include "lee_control_zmq.h"

#include <volasim_msgs/DroneState.pb.h>
#include <volasim_msgs/Thrust.pb.h>
#include <volasim_msgs/Trajectory.pb.h>

#include <iostream>

namespace {

bool ends_with(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}  // namespace

LeeControlZmq::LeeControlZmq(double control_dt)
    : ctx_(1),
      state_sub_(ctx_, zmq::socket_type::sub),
      cmd_pub_(ctx_, zmq::socket_type::pub),
      cmd_pos_pull_(ctx_, zmq::socket_type::pull),
      traj_sub_(ctx_, zmq::socket_type::sub),
      control_dt_(control_dt) {
  params_["kp"]       = 3.5;
  params_["kv"]       = 2.1;
  params_["kR"]       = 1.0;
  params_["kw"]       = 0.1;
  params_["mass"]     = 0.68;
  params_["length"]   = 0.17;
  params_["c_torque"] = 0.016;
  params_["j0"]       = 0.007;
  params_["j1"]       = 0.007;
  params_["j2"]       = 0.012;

  controller_.loadParams(params_);
}

LeeControlZmq::~LeeControlZmq() {
  stop();
}

void LeeControlZmq::run() {
  state_sub_.connect("ipc:///tmp/volasim_state");
  state_sub_.set(zmq::sockopt::subscribe, "");
  state_sub_.set(zmq::sockopt::rcvtimeo, 100);

  cmd_pub_.bind("tcp://*:5557");

  cmd_pos_pull_.bind("ipc:///tmp/volasim_cmd_pos");
  cmd_pos_pull_.bind("tcp://*:5558");
  cmd_pos_pull_.set(zmq::sockopt::rcvtimeo, 0);

  traj_sub_.bind("ipc:///tmp/volasim_traj");
  traj_sub_.bind("tcp://*:5560");
  traj_sub_.set(zmq::sockopt::subscribe, "");
  traj_sub_.set(zmq::sockopt::rcvtimeo, 100);

  running_     = true;
  recv_thread_ = std::thread([this] { receiveLoop(); });
  traj_thread_ = std::thread([this] { pollTrajectory(); });
  controlLoop();
}

void LeeControlZmq::stop() {
  running_ = false;
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (traj_thread_.joinable()) {
    traj_thread_.join();
  }
}

void LeeControlZmq::receiveLoop() {
  while (running_.load()) {
    zmq::message_t topic_frame;
    auto           result = state_sub_.recv(topic_frame);
    if (!result.has_value()) {
      continue;
    }

    std::string topic(static_cast<const char*>(topic_frame.data()),
                      topic_frame.size());

    if (!state_sub_.get(zmq::sockopt::rcvmore)) {
      continue;
    }

    zmq::message_t data_frame;
    (void)state_sub_.recv(data_frame);

    if (!ends_with(topic, "/state")) {
      continue;
    }

    volasim_msgs::DroneState drone_state;
    if (!drone_state.ParseFromArray(data_frame.data(),
                                    static_cast<int>(data_frame.size()))) {
      continue;
    }

    const auto&        odom = drone_state.odom();
    Eigen::Quaterniond q(odom.orientation().w(), odom.orientation().x(),
                         odom.orientation().y(), odom.orientation().z());

    std::lock_guard<std::mutex> lock(state_mtx_);
    state_.pos = {odom.position().x(), odom.position().y(),
                  odom.position().z()};
    state_.vel = {odom.linvel().x(), odom.linvel().y(), odom.linvel().z()};
    state_.w   = {odom.angvel().x(), odom.angvel().y(), odom.angvel().z()};
    state_.rot = q.toRotationMatrix();

    if (!initialized_.load()) {
      initialized_ = true;
      std::cout << "[lee_control_zmq] state received, position: "
                << state_.pos.transpose() << std::endl;
    }
  }
}

void LeeControlZmq::pollTrajectory() {
  while (running_.load()) {
    zmq::message_t msg;
    auto           result = traj_sub_.recv(msg);
    if (!result.has_value()) {
      continue;
    }

    volasim_msgs::Trajectory traj_proto;
    if (!traj_proto.ParseFromArray(msg.data(), static_cast<int>(msg.size()))) {
      continue;
    }

    if (traj_proto.points_size() == 0) {
      continue;
    }

    vola::trajectory_t traj;
    traj.states.reserve(traj_proto.points_size());
    for (const auto& pt : traj_proto.points()) {
      vola::state_t s;
      s.pos  = {pt.pos().x(), pt.pos().y(), pt.pos().z()};
      s.vel  = {pt.vel().x(), pt.vel().y(), pt.vel().z()};
      s.acc  = {pt.acc().x(), pt.acc().y(), pt.acc().z()};
      s.jerk = {pt.jerk().x(), pt.jerk().y(), pt.jerk().z()};
      s.yaw  = pt.yaw();
      s.time = pt.time();

      if (pt.has_orientation()) {
        Eigen::Quaterniond q(pt.orientation().w(), pt.orientation().x(),
                             pt.orientation().y(), pt.orientation().z());
        s.rot = q.toRotationMatrix();
      }
      if (pt.has_angular_vel()) {
        s.w = {pt.angular_vel().x(), pt.angular_vel().y(),
               pt.angular_vel().z()};
      }
      if (pt.has_angular_acc()) {
        s.w_dot = {pt.angular_acc().x(), pt.angular_acc().y(),
                   pt.angular_acc().z()};
      }

      traj.states.push_back(s);
    }

    {
      std::lock_guard<std::mutex> lock(traj_mtx_);
      active_traj_ = std::move(traj);
      traj_start_  = std::chrono::steady_clock::now();
    }
    traj_set_ = true;
    std::cout << "[lee_control_zmq] trajectory received ("
              << traj_proto.points_size() << " points)\n";
  }
}

void LeeControlZmq::pullDesiredState() {
  zmq::message_t cmd_msg;
  if (cmd_pos_pull_.recv(cmd_msg).has_value()) {
    volasim_msgs::DroneState cmd_state;
    if (cmd_state.ParseFromArray(cmd_msg.data(),
                                 static_cast<int>(cmd_msg.size()))) {
      const auto&     pos = cmd_state.odom().position();
      Eigen::Vector3d target(pos.x(), pos.y(), pos.z());

      vola::state_t local;
      {
        std::lock_guard<std::mutex> lock(state_mtx_);
        local = state_;
      }

      auto minjerk = traj_gen_.get_trajectory(local.pos, target, 2.0);
      {
        std::lock_guard<std::mutex> lock(traj_mtx_);
        active_traj_ = minjerk;
        traj_start_  = std::chrono::steady_clock::now();
      }
      traj_set_ = true;
      std::cout << "[lee_control_zmq] trajectory set: " << target.transpose()
                << '\n';
    }
  }
}

void LeeControlZmq::controlLoop() {
  auto next = std::chrono::steady_clock::now();
  auto step = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(control_dt_));

  while (running_.load()) {
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                              last_tick_)
                    .count();
    if (dt > 2 * control_dt_) {
      std::cout << "[lee_control_zmq] dt was: " << dt << std::endl;
    }

    last_tick_ = std::chrono::steady_clock::now();

    if (initialized_.load()) {
      pullDesiredState();
    }

    if (!initialized_.load() || !traj_set_.load()) {
      next += step;
      std::this_thread::sleep_until(next);
      continue;
    }

    vola::state_t desired;
    {
      std::lock_guard<std::mutex> lock(traj_mtx_);
      double                      t = std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - traj_start_)
                     .count();
      desired = active_traj_.at_time(t);
    }

    vola::state_t local;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      local = state_;
    }

    Eigen::Vector4d cmd = controller_.computeControls(local, desired);

    volasim_msgs::Thrust thrust;
    thrust.set_f1(static_cast<float>(cmd[0]));
    thrust.set_f2(static_cast<float>(cmd[1]));
    thrust.set_f3(static_cast<float>(cmd[2]));
    thrust.set_f4(static_cast<float>(cmd[3]));

    std::string bytes;
    (void)thrust.SerializeToString(&bytes);
    cmd_pub_.send(zmq::buffer(bytes), zmq::send_flags::none);

    next += step;
    std::this_thread::sleep_until(next);
  }
}

int main() {
  LeeControlZmq controller;
  controller.run();
  return 0;
}
