#include "lee_control_zmq.h"

#include <volasim_msgs/DroneState.pb.h>
#include <volasim_msgs/Thrust.pb.h>
#include <volasim_msgs/Trajectory.pb.h>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <string>
#include <vector>

namespace {

// The frame volasim-recorder publishes the drone pose in.
const std::string kFrame = "drone_0/odom";

double deg(double rad) {
  return rad * 180. / M_PI;
}

// ZYX order.
Eigen::Vector3d rpyDeg(const Eigen::Matrix3d& R) {
  return {deg(std::atan2(R(2, 1), R(2, 2))),
          deg(std::asin(std::clamp(-R(2, 0), -1., 1.))),
          deg(std::atan2(R(1, 0), R(0, 0)))};
}

double angleDeg(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return deg(std::acos(std::clamp(a.dot(b), -1., 1.)));
}

bool ends_with(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}  // namespace

LeeControlZmq::LeeControlZmq(double control_dt)
    : ctx_(1),
      state_sub_(ctx_, zmq::socket_type::sub),
      cmd_pub_(ctx_, "/lee_control/thrust",
               blackbox::Publisher::Framing::Payload),
      cmd_pos_pull_(ctx_, zmq::socket_type::pull),
      traj_sub_(ctx_, zmq::socket_type::sub),
      control_dt_(control_dt) {
  params_["kp"]       = 3.5;
  params_["kv"]       = 2.1;
  params_["kR"]       = 3.0;
  params_["kw"]       = 0.35;
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

  traj_sub_.connect("ipc:///tmp/volasim_traj");
  traj_sub_.connect("tcp://localhost:5560");
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
      BLACKBOX_INFO << "state received, position: " << state_.pos.transpose();
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
      s.pos      = {pt.pos().x(), pt.pos().y(), pt.pos().z()};
      s.vel      = {pt.vel().x(), pt.vel().y(), pt.vel().z()};
      s.acc      = {pt.acc().x(), pt.acc().y(), pt.acc().z()};
      s.jerk     = {pt.jerk().x(), pt.jerk().y(), pt.jerk().z()};
      s.yaw      = pt.yaw();
      s.yaw_rate = pt.yaw_dot();
      s.time     = pt.time();

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
    drawTrajectory(traj);

    {
      std::lock_guard<std::mutex> lock(traj_mtx_);
      active_traj_ = std::move(traj);
      traj_start_  = std::chrono::steady_clock::now();
      ++traj_seq_;
    }
    traj_set_ = true;
    BLACKBOX_DEBUG << "trajectory received (" << traj_proto.points_size()
                   << " points)";
  }
}

void LeeControlZmq::pullDesiredState() {
  zmq::message_t cmd_msg;
  if (cmd_pos_pull_.recv(cmd_msg).has_value()) {
    volasim_msgs::DroneState cmd_state;
    if (cmd_state.ParseFromArray(cmd_msg.data(),
                                 static_cast<int>(cmd_msg.size()))) {
      blackbox::record("/lee_control/waypoint", cmd_state);
      const auto&     pos = cmd_state.odom().position();
      Eigen::Vector3d target(pos.x(), pos.y(), pos.z());

      vola::state_t local;
      {
        std::lock_guard<std::mutex> lock(state_mtx_);
        local = state_;
      }

      auto minjerk = traj_gen_.get_trajectory(local.pos, target, 2.0);
      drawTrajectory(minjerk);
      {
        std::lock_guard<std::mutex> lock(traj_mtx_);
        active_traj_ = minjerk;
        traj_start_  = std::chrono::steady_clock::now();
        ++traj_seq_;
      }
      traj_set_ = true;
      BLACKBOX_INFO << "waypoint received, min-jerk trajectory to "
                    << target.transpose();
    }
  }
}

void LeeControlZmq::controlLoop() {
  auto next = std::chrono::steady_clock::now();
  auto step = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(control_dt_));

  while (running_.load()) {
    auto   now = std::chrono::steady_clock::now();
    double lag = std::chrono::duration<double>(now - next).count();

    last_tick_ = now;

    if (initialized_.load()) {
      pullDesiredState();
    }

    if (!initialized_.load() || !traj_set_.load()) {
      next += step;
      std::this_thread::sleep_until(next);
      continue;
    }

    tick_t tick;
    tick.lag_s = lag;
    {
      std::lock_guard<std::mutex> lock(traj_mtx_);
      tick.traj_t = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - traj_start_)
                        .count();
      tick.desired = active_traj_.at_time(tick.traj_t);
      tick.traj_end =
          active_traj_.states.empty() ? 0. : active_traj_.states.back().time;
      tick.traj_seq = traj_seq_;
    }

    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      tick.local = state_;
    }

    tick.cmd = controller_.computeControls(tick.local, tick.desired);
    tick.dbg = controller_.debug();

    volasim_msgs::Thrust thrust;
    thrust.set_f1(static_cast<float>(tick.cmd[0]));
    thrust.set_f2(static_cast<float>(tick.cmd[1]));
    thrust.set_f3(static_cast<float>(tick.cmd[2]));
    thrust.set_f4(static_cast<float>(tick.cmd[3]));

    tick.stamp_ns = blackbox::nowNs();
    cmd_pub_.send(thrust, tick.stamp_ns);
    recordTick(tick);
    logEvents(tick);
    prev_tick_     = tick;
    has_prev_tick_ = true;

    next += step;
    std::this_thread::sleep_until(next);
  }
}

void LeeControlZmq::drawTrajectory(const vola::trajectory_t& traj) {
  if (traj.states.empty()) {
    return;
  }
  std::vector<Eigen::Vector3d> path;
  path.reserve(traj.states.size());
  for (const auto& s : traj.states) {
    path.push_back(s.pos);
  }
  blackbox::Scene scene(kFrame);
  scene.line(path, blackbox::color::cyan, 0.03)
      .sphere(path.back(), 0.2, blackbox::color::green);
  blackbox::draw("/lee_control/trajectory", scene);
}

void LeeControlZmq::recordTick(const tick_t& tick) {
  const vola::state_t&     desired = tick.desired;
  const vola::state_t&     local   = tick.local;
  const vola::lee_debug_t& dbg     = tick.dbg;

  const Eigen::Vector3d e_pos = desired.pos - local.pos;
  const Eigen::Vector3d e_vel = desired.vel - local.vel;
  const Eigen::Vector3d rpy   = rpyDeg(local.rot);
  const double          yaw   = std::atan2(local.rot(1, 0), local.rot(0, 0));
  blackbox::plot("/lee_control/tracking",
                 {{"t", tick.traj_t},
                  {"lag_ms", tick.lag_s * 1e3},
                  {"x_des", desired.pos.x()},
                  {"y_des", desired.pos.y()},
                  {"z_des", desired.pos.z()},
                  {"x", local.pos.x()},
                  {"y", local.pos.y()},
                  {"z", local.pos.z()},
                  {"ex", e_pos.x()},
                  {"ey", e_pos.y()},
                  {"ez", e_pos.z()},
                  {"e_pos", e_pos.norm()},
                  {"vx_des", desired.vel.x()},
                  {"vy_des", desired.vel.y()},
                  {"vz_des", desired.vel.z()},
                  {"vx", local.vel.x()},
                  {"vy", local.vel.y()},
                  {"vz", local.vel.z()},
                  {"e_vel", e_vel.norm()},
                  {"yaw_des", desired.yaw},
                  {"yaw", yaw}},
                 tick.stamp_ns);

  blackbox::plot("/lee_control/inputs",
                 {{"ax_des", desired.acc.x()},
                  {"ay_des", desired.acc.y()},
                  {"az_des", desired.acc.z()},
                  {"jx_des", desired.jerk.x()},
                  {"jy_des", desired.jerk.y()},
                  {"jz_des", desired.jerk.z()},
                  {"yaw_rate_des", desired.yaw_rate},
                  {"roll_deg", rpy.x()},
                  {"pitch_deg", rpy.y()},
                  {"yaw_deg", rpy.z()},
                  {"wx", local.w.x()},
                  {"wy", local.w.y()},
                  {"wz", local.w.z()},
                  {"traj_seq", static_cast<double>(tick.traj_seq)},
                  {"traj_end", tick.traj_end},
                  {"past_end", tick.traj_t > tick.traj_end ? 1. : 0.}},
                 tick.stamp_ns);

  const Eigen::Vector3d body_z = local.rot.col(2);
  const Eigen::Vector3d torque = tick.cmd.tail<3>();
  blackbox::plot(
      "/lee_control/internals",
      {{"fz", tick.cmd[0]},
       {"tau_x", torque.x()},
       {"tau_y", torque.y()},
       {"tau_z", torque.z()},
       {"tau_norm", torque.norm()},
       {"tauR_x", dbg.torque_R.x()},
       {"tauR_y", dbg.torque_R.y()},
       {"tauR_z", dbg.torque_R.z()},
       {"tauw_x", dbg.torque_w.x()},
       {"tauw_y", dbg.torque_w.y()},
       {"tauw_z", dbg.torque_w.z()},
       {"pid_x", dbg.pid_term.x()},
       {"pid_y", dbg.pid_term.y()},
       {"pid_z", dbg.pid_term.z()},
       {"pid_norm", dbg.pid_norm},
       {"b3d_x", dbg.b3d.x()},
       {"b3d_y", dbg.b3d.y()},
       {"b3d_z", dbg.b3d.z()},
       {"tilt_des_deg", deg(std::acos(std::clamp(dbg.b3d.z(), -1., 1.)))},
       {"tilt_deg", deg(std::acos(std::clamp(body_z.z(), -1., 1.)))},
       {"thrust_dir_err_deg", angleDeg(dbg.b3d, body_z)},
       {"psi", dbg.psi},
       {"flipped", dbg.flipped ? 1. : 0.},
       {"cutoff", dbg.cutoff ? 1. : 0.},
       {"eR_x", dbg.eR.x()},
       {"eR_y", dbg.eR.y()},
       {"eR_z", dbg.eR.z()},
       {"ew_x", dbg.ew.x()},
       {"ew_y", dbg.ew.y()},
       {"ew_z", dbg.ew.z()},
       {"wdes_x", dbg.desired_w.x()},
       {"wdes_y", dbg.desired_w.y()},
       {"wdes_z", dbg.desired_w.z()},
       {"hw_x", dbg.h_w.x()},
       {"hw_y", dbg.h_w.y()},
       {"hw_z", dbg.h_w.z()}},
      tick.stamp_ns);

  blackbox::Scene scene(kFrame);
  scene.sphere(desired.pos, 0.12, blackbox::color::yellow)
      .line(std::vector<Eigen::Vector3d>{local.pos, desired.pos},
            blackbox::color::red, 0.02);
  blackbox::draw("/lee_control/desired", scene, tick.stamp_ns);

  blackbox::Scene attitude(kFrame);
  attitude
      .line(std::vector<Eigen::Vector3d>{local.pos, local.pos + 0.5 * dbg.b3d},
            blackbox::color::orange, 0.02)
      .line(std::vector<Eigen::Vector3d>{local.pos, local.pos + 0.5 * body_z},
            blackbox::color::white, 0.02);
  blackbox::draw("/lee_control/attitude", attitude, tick.stamp_ns);
}

void LeeControlZmq::logEvents(const tick_t& tick) {
  if (!has_prev_tick_) {
    return;
  }
  const tick_t&            prev = prev_tick_;
  const vola::lee_debug_t& dbg  = tick.dbg;

  if (dbg.flipped != prev.dbg.flipped) {
    BLACKBOX_WARN << "desired heading flip (R_dw2) "
                  << (dbg.flipped ? "on" : "off") << " at t " << tick.traj_t
                  << " s of trajectory #" << tick.traj_seq;
  }
  if (dbg.cutoff != prev.dbg.cutoff) {
    BLACKBOX_WARN << "thrust cutoff " << (dbg.cutoff ? "on" : "off") << " at t "
                  << tick.traj_t << " s of trajectory #" << tick.traj_seq;
  }
  if ((tick.cmd[0] < 0.) != (prev.cmd[0] < 0.)) {
    BLACKBOX_WARN << "commanded thrust "
                  << (tick.cmd[0] < 0. ? "negative" : "positive again")
                  << " at t " << tick.traj_t << " s of trajectory #"
                  << tick.traj_seq;
  }
  // Single-sample trajectories (a planner holding at its goal) are always past their end.
  if (tick.traj_seq == prev.traj_seq && tick.traj_end > 0. &&
      tick.traj_t > tick.traj_end && prev.traj_t <= prev.traj_end) {
    BLACKBOX_DEBUG << "reached the end of trajectory #" << tick.traj_seq;
  }
}

namespace {
LeeControlZmq* g_controller = nullptr;
void           signal_handler(int) {
  if (g_controller) {
    g_controller->requestStop();
  }
}
}  // namespace

int main() {
  blackbox::init("lee_control_zmq");
  LeeControlZmq controller;
  g_controller = &controller;
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  controller.run();
  blackbox::shutdown();
  return 0;
}
