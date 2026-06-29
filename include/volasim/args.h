#ifndef ARGS_H
#define ARGS_H

#include <stdexcept>
#include <string>

struct Args {
  std::string world_path{"./definitions/worlds/world_250_world.xml"};
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
    }
  }
  return args;
}

#endif
