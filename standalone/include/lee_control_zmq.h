#ifndef STANDALONE_LEE_CONTROL_ZMQ_H
#define STANDALONE_LEE_CONTROL_ZMQ_H

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string_view>
#include <thread>
#include <unordered_map>

#include <blackbox/blackbox.h>
#include <Eigen/Core>
#include <zmq.hpp>

#include "lee_controller.h"
#include "minjerk_generator.h"
#include "types.h"

class LeeControlZmq {
 public:
  LeeControlZmq(double control_dt = 0.01);
  ~LeeControlZmq();

  LeeControlZmq(LeeControlZmq&)                   = delete;
  LeeControlZmq(LeeControlZmq&&)                  = delete;
  LeeControlZmq& operator=(LeeControlZmq& other)  = delete;
  LeeControlZmq& operator=(LeeControlZmq&& other) = delete;

  void run();
  void stop();

  void requestStop() { running_ = false; }

 private:
  struct tick_t {
    double            traj_t   = 0.;
    double            traj_end = 0.;
    uint64_t          traj_seq = 0;
    double            lag_s    = 0.;
    int64_t           stamp_ns = 0;
    vola::state_t     desired;
    vola::state_t     local;
    Eigen::Vector4d   cmd = Eigen::Vector4d::Zero();  // thrust, torque xyz
    vola::lee_debug_t dbg;
  };

  void receiveLoop();
  void controlLoop();
  void pullDesiredState();
  void pollTrajectory();
  void drawTrajectory(const vola::trajectory_t& traj);
  void recordTick(const tick_t& tick);
  void logEvents(const tick_t& tick);

  zmq::context_t      ctx_;
  zmq::socket_t       state_sub_;
  blackbox::Publisher cmd_pub_;
  zmq::socket_t       cmd_pos_pull_;
  zmq::socket_t       traj_sub_;

  vola::state_t state_;
  std::mutex    state_mtx_;

  std::atomic_bool initialized_{false};
  std::atomic_bool traj_set_{false};
  std::atomic_bool running_{false};

  vola::LeeController controller_;
  MinJerkGenerator    traj_gen_;
  vola::trajectory_t  active_traj_;
  uint64_t            traj_seq_ = 0;  // guarded by traj_mtx_
  std::mutex          traj_mtx_;

  // Control thread only.
  tick_t prev_tick_;
  bool   has_prev_tick_ = false;

  std::chrono::steady_clock::time_point traj_start_;
  std::chrono::steady_clock::time_point last_tick_;
  double                                control_dt_;

  std::unordered_map<std::string_view, double> params_;

  std::thread recv_thread_;
  std::thread traj_thread_;
};

#endif
