#ifndef RATE_COUNTER_H
#define RATE_COUNTER_H

#include <chrono>
#include <iostream>
#include <string>
#include <utility>

// Reports how fast a loop is actually running, once per window. A loop that
// falls behind its target shows up here as a lower rate, since a missed
// iteration is simply one that never ticked.
class RateCounter {
 public:
  explicit RateCounter(std::string name, std::string unit = "Hz",
                       double window_seconds = 1.)
      : name_(std::move(name)),
        unit_(std::move(unit)),
        window_seconds_(window_seconds),
        window_start_(Clock::now()) {}

  // Starts a fresh window; call when the loop actually begins so the first
  // report does not average in the time spent getting there.
  void reset() {
    ticks_        = 0;
    window_start_ = Clock::now();
  }

  void tick() {
    ++ticks_;

    const Clock::time_point now = Clock::now();
    const double            elapsed =
        std::chrono::duration<double>(now - window_start_).count();

    if (elapsed < window_seconds_) {
      return;
    }

    std::cout << "[" << name_ << "] " << ticks_ / elapsed << " " << unit_
              << "\n";

    ticks_        = 0;
    window_start_ = now;
  }

 private:
  using Clock = std::chrono::steady_clock;

  std::string name_;
  std::string unit_;

  double            window_seconds_;
  Clock::time_point window_start_;

  unsigned int ticks_{0};
};

#endif
