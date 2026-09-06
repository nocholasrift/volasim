#include "lee_control_zmq.h"

#include <volasim/comms/msgs/DroneState.pb.h>
#include <volasim/comms/msgs/Thrust.pb.h>

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

  running_     = true;
  recv_thread_ = std::thread([this] { receiveLoop(); });
  controlLoop();
}

void LeeControlZmq::stop() {
  running_ = false;
  if (recv_thread_.joinable()) {
    recv_thread_.join();
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
    if (!drone_state.ParseFromArray(data_frame.data(), data_frame.size())) {
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

void LeeControlZmq::controlLoop() {
  auto next = std::chrono::steady_clock::now();
  auto step = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(control_dt_));

  while (running_.load()) {
    if (initialized_.load() && !traj_set_.load()) {
      vola::state_t local;
      {
        std::lock_guard<std::mutex> lock(state_mtx_);
        local = state_;
      }
      Eigen::Vector3d target(local.pos.x(), local.pos.y(), 1.0);
      active_traj_ = traj_gen_.get_trajectory(local.pos, target, 2.0);
      traj_start_  = std::chrono::steady_clock::now();
      traj_set_    = true;
      std::cout << "[lee_control_zmq] trajectory set: hover at "
                << target.transpose() << std::endl;
    }

    if (!initialized_.load() || !traj_set_.load()) {
      next += step;
      std::this_thread::sleep_until(next);
      continue;
    }

    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                             traj_start_)
                   .count();

    size_t ind = 0;
    for (size_t i = 0; i < active_traj_.states.size(); ++i) {
      if (t < active_traj_.states[i].time) {
        break;
      }
      ind = i;
    }

    const vola::state_t& desired = active_traj_.states[ind];

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
    thrust.SerializeToString(&bytes);
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
