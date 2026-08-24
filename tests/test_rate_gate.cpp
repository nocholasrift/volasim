#include <doctest.h>

#include <volasim/simulation/rate_gate.h>

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace {

using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

}  // namespace

TEST_CASE("fires immediately at start, then holds until a period elapses") {
  const Clock::time_point t0;
  RateGate                gate(10., t0);  // 100 ms period

  CHECK(gate.due(t0));  // first poll is due
  CHECK_FALSE(gate.due(t0 + milliseconds(50)));
  CHECK(gate.due(t0 + milliseconds(100)));
}

TEST_CASE("fires once per period when polled faster than the rate") {
  const Clock::time_point t0;
  RateGate                gate(100., t0);  // 10 ms period

  int fires = 0;
  for (int ms = 0; ms <= 100; ++ms) {
    if (gate.due(t0 + milliseconds(ms))) {
      ++fires;
    }
  }

  CHECK(fires == 11);  // ticks at 0, 10, 20, ... 100
}

TEST_CASE("falling behind drops missed ticks rather than bursting") {
  const Clock::time_point t0;
  RateGate                gate(100., t0);  // 10 ms period

  CHECK(gate.due(t0));

  // a long stall past many periods yields a single tick, not one per miss
  CHECK(gate.due(t0 + milliseconds(1000)));

  // and the schedule resumes one period out from the stall, not from 10 ms
  CHECK_FALSE(gate.due(t0 + milliseconds(1005)));
  CHECK(gate.due(t0 + milliseconds(1010)));
}

TEST_CASE("degenerate rates are rejected") {
  const Clock::time_point t0;

  CHECK_THROWS_AS(RateGate(0., t0), std::invalid_argument);
  CHECK_THROWS_AS(RateGate(-1., t0), std::invalid_argument);
  CHECK_THROWS_AS(RateGate(std::nan(""), t0), std::invalid_argument);
  CHECK_THROWS_AS(RateGate(std::numeric_limits<double>::infinity(), t0),
                  std::invalid_argument);
}
