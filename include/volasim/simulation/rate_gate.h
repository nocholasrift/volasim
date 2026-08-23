#ifndef RATE_GATE_H
#define RATE_GATE_H

#include <chrono>
#include <cmath>
#include <stdexcept>

// Poll-style rate limiter for work driven from a faster loop: the render loop
// calls due(now) every frame and gets true only when a tick is owed. Unlike
// LoopPacer — which advances its schedule on every call because its loop always
// waits out the deadline — this advances only when it actually fires, so it is
// safe to poll every frame. Falling behind drops the missed ticks rather than
// bursting to catch up, so a hitch never triggers a flurry of captures.
class RateGate {
 public:
  using Clock = std::chrono::steady_clock;

  RateGate(double hz, Clock::time_point start)
      : period_(checkedPeriod(hz)), next_(start) {}

  bool due(Clock::time_point now) {
    if (now < next_) {
      return false;
    }

    next_ += period_;
    if (next_ <= now) {  // fell a whole period behind: drop the missed ticks
      next_ = now + period_;
    }
    return true;
  }

  [[nodiscard]] Clock::duration period() const { return period_; }

 private:
  static Clock::duration checkedPeriod(double hz) {
    if (!std::isfinite(hz) || hz <= 0.) {
      throw std::invalid_argument("[RateGate] hz must be finite and positive");
    }

    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1. / hz));

    if (period <= Clock::duration::zero()) {
      throw std::invalid_argument("[RateGate] hz is too high to represent");
    }
    return period;
  }

  Clock::duration   period_;
  Clock::time_point next_;
};

#endif
