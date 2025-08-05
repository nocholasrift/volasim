#ifndef VOLASIM_TYPES_H
#define VOLASIM_TYPES_H

#include <mutex>
#include <string_view>

struct SimState {
  std::string state;
  std::mutex mutex;
};

#endif
