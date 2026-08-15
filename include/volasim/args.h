#ifndef VOLASIM_ARGS_H
#define VOLASIM_ARGS_H

#include <stdexcept>
#include <string>

struct Args {
  std::string world_path{"./definitions/worlds/world_250_world.xml"};

  // rate of the physics thread, independent of the render loop's fps
  double physics_hz{1000.};

  // print the rate each loop actually achieves, once per second
  bool report_rates{false};
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
      args.physics_hz = std::stod(argv[++i]);
      if (args.physics_hz <= 0.) {
        throw std::runtime_error("--physics-hz must be > 0");
      }
    } else if (arg == "--rates") {
      args.report_rates = true;
    }
  }
  return args;
}

#endif
