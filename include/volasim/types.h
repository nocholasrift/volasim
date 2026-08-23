#ifndef VOLASIM_TYPES_H
#define VOLASIM_TYPES_H

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

// Latest serialized DroneState per drone, keyed by drone id.
struct SimState {
  std::unordered_map<uint32_t, std::string> states;
  std::mutex                                mutex;
};

#endif
