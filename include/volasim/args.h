#ifndef VOLASIM_ARGS_H
#define VOLASIM_ARGS_H

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

// Bounds on --physics-hz. The step period is held as a steady_clock::duration,
// so a rate above the maximum truncates to a zero-length step (the loop then
// free-runs) and one below the minimum overflows the duration.
inline constexpr double kMinPhysicsHz = 1e-3;
inline constexpr double kMaxPhysicsHz = 1e6;

struct Args {
  std::string world_path{"./definitions/worlds/world_250_world.xml"};

  // rate of the physics thread, independent of the render loop's fps
  double physics_hz{1000.};

  // print the rate each loop actually achieves, once per second
  bool report_rates{false};

  // draw poses blended between the last two physics steps rather than the
  // newest one as it stands
  bool interpolate{false};
};

// Parses a rate flag's value and bounds-checks it. Shared by --physics-hz and
// --sensor-hz so both reject the same degenerate inputs (non-numeric, trailing
// junk, non-finite, out of range) with matching messages.
inline double parseRate(const std::string& flag, const std::string& value,
                        double min_hz, double max_hz) {
  std::size_t consumed = 0;
  double      rate     = 0.;
  bool        parsed   = true;

  try {
    rate = std::stod(value, &consumed);
  } catch (const std::exception&) {
    parsed = false;
  }

  // stod stops at the first character it cannot use, so without the length
  // check a value like "100abc" would be taken as 100
  if (!parsed || consumed != value.size()) {
    throw std::runtime_error(flag + " expects a number, got '" + value + "'");
  }

  // isfinite first: NaN compares false against any bound
  if (!std::isfinite(rate) || rate < min_hz || rate > max_hz) {
    std::ostringstream err_msg;
    err_msg << flag << " must be a finite rate between " << min_hz << " and "
            << max_hz << " Hz, got '" << value << "'";
    throw std::runtime_error(err_msg.str());
  }

  return rate;
}

inline Args parseArgs(int argc, char* argv[]) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--world" || arg == "-w") {
      if (i + 1 >= argc) {
        throw std::runtime_error(arg + " requires a path argument");
      }
      args.world_path = argv[++i];
    } else if (arg == "--physics-hz") {
      if (i + 1 >= argc) {
        throw std::runtime_error(arg + " requires a rate in Hz");
      }
      args.physics_hz =
          parseRate("--physics-hz", argv[++i], kMinPhysicsHz, kMaxPhysicsHz);
    } else if (arg == "--rates") {
      args.report_rates = true;
    } else if (arg == "--interpolate") {
      args.interpolate = true;
    }
  }
  return args;
}

#endif
