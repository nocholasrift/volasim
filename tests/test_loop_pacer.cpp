#include <doctest.h>

#include <volasim/simulation/loop_pacer.h>

namespace {

using Clock = LoopPacer::Clock;

Clock::time_point at(double seconds) {
  return Clock::time_point{} + std::chrono::duration_cast<Clock::duration>(
                                   std::chrono::duration<double>(seconds));
}

double secondsBetween(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double>(b - a).count();
}

}  // namespace

// Driven by a clock the test controls, so this says what the loop schedules
// rather than what the machine happened to manage.
TEST_CASE("a loop that keeps up runs at exactly its rate") {
  constexpr double  kStep = 1. / 1000.;
  LoopPacer         pacer(kStep, at(0.));
  Clock::time_point now = at(0.);

  for (int step = 1; step <= 1000; ++step) {
    const Clock::time_point deadline = pacer.nextDeadline(now);

    CHECK(secondsBetween(at(0.), deadline) == doctest::Approx(step * kStep));

    now = deadline;  // a loop that wakes exactly on time
  }

  CHECK(pacer.droppedSteps() == 0);
  CHECK(secondsBetween(at(0.), now) == doctest::Approx(1.));
}

TEST_CASE("work taking part of the step does not shift the schedule") {
  constexpr double kStep = 0.01;
  LoopPacer        pacer(kStep, at(0.));

  // each iteration wakes on time, then spends 40% of the period working
  Clock::time_point now = at(0.);
  for (int step = 1; step <= 50; ++step) {
    const Clock::time_point deadline = pacer.nextDeadline(now);
    CHECK(secondsBetween(at(0.), deadline) == doctest::Approx(step * kStep));
    now = deadline + std::chrono::duration_cast<Clock::duration>(
                         std::chrono::duration<double>(kStep * 0.4));
  }

  CHECK(pacer.droppedSteps() == 0);
}

// Chasing missed steps would only push the loop further behind, so it gives up
// on them and reschedules from the present.
TEST_CASE("a loop that falls behind drops the missed steps") {
  constexpr double kStep = 0.01;
  LoopPacer        pacer(kStep, at(0.));

  pacer.nextDeadline(at(0.));
  CHECK(pacer.droppedSteps() == 0);

  // the caller vanishes for ten steps
  const Clock::time_point deadline = pacer.nextDeadline(at(0.105));

  CHECK(pacer.droppedSteps() >= 9);
  CHECK(secondsBetween(at(0.105), deadline) == doctest::Approx(kStep));
}

TEST_CASE("deadlines are never handed out in the past") {
  LoopPacer pacer(0.01, at(0.));

  Clock::time_point now = at(0.);
  for (int i = 0; i < 100; ++i) {
    now                              = now + std::chrono::milliseconds(7);
    const Clock::time_point deadline = pacer.nextDeadline(now);
    CHECK(deadline > now);
  }
}
