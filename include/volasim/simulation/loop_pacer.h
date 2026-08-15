#ifndef LOOP_PACER_H
#define LOOP_PACER_H

#include <chrono>

// Keeps a loop on a fixed schedule by handing out the deadline for each
// iteration. It owns the schedule but does no waiting of its own: the caller
// passes in the current time and decides how to wait, so the physics thread can
// wait on its condition variable and a test can drive the whole thing from a
// clock it controls.
class LoopPacer {
 public:
  using Clock = std::chrono::steady_clock;

  LoopPacer(double step_seconds, Clock::time_point start)
      : step_(std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(step_seconds))),
        next_(start) {}

  // Advances the schedule and returns when the next iteration is due. Falling
  // behind drops the missed steps rather than chasing them, which would only
  // push the loop further behind.
  Clock::time_point nextDeadline(Clock::time_point now) {
    next_ += step_;

    if (next_ < now) {
      dropped_ += static_cast<unsigned int>((now - next_) / step_) + 1;
      next_     = now + step_;
    }

    return next_;
  }

  [[nodiscard]] Clock::duration step() const { return step_; }

  // Steps skipped because the loop could not keep up. A loop holding its rate
  // leaves this at zero.
  [[nodiscard]] unsigned int droppedSteps() const { return dropped_; }

 private:
  Clock::duration   step_;
  Clock::time_point next_;

  unsigned int dropped_{0};
};

#endif
