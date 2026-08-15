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

      const std::string value = argv[++i];
      try {
        args.physics_hz = std::stod(value);
      } catch (const std::exception&) {
        throw std::runtime_error("--physics-hz expects a number, got '" +
                                 value + "'");
      }

      // isfinite first: NaN compares false against any bound
      if (!std::isfinite(args.physics_hz) || args.physics_hz < kMinPhysicsHz ||
          args.physics_hz > kMaxPhysicsHz) {
        std::ostringstream err_msg;
        err_msg << "--physics-hz must be a finite rate between "
                << kMinPhysicsHz << " and " << kMaxPhysicsHz << " Hz, got '"
                << value << "'";
        throw std::runtime_error(err_msg.str());
      }
    } else if (arg == "--rates") {
      args.report_rates = true;
    } else if (arg == "--interpolate") {
      args.interpolate = true;
    }
  }
  return args;
}

#endif
