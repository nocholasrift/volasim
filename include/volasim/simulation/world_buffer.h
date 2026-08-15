#ifndef WORLD_BUFFER_H
#define WORLD_BUFFER_H

#include <volasim/simulation/entity.h>
#include <volasim/simulation/transform.h>

#include <chrono>
#include <mutex>
#include <vector>

// Poses of every physics-driven entity at one instant, indexed by EntityID.
// Entities without an entry are not driven by physics and keep the transform
// they were authored with.
class WorldSnapshot {
 public:
  void set(EntityID id, const Transform& transform);

  // nullptr when the entity has no physics-driven pose in this snapshot
  [[nodiscard]] const Transform* find(EntityID id) const;

  // Fills this snapshot with prev and curr blended by alpha. Entities that
  // appear only in curr are taken as-is: there is nothing to blend from.
  void blendFrom(const WorldSnapshot& prev, const WorldSnapshot& curr,
                 float alpha);

  // Drops every entry but keeps the storage, so a steady-state physics step
  // never allocates.
  void invalidate();

  void swap(WorldSnapshot& other) noexcept { slots_.swap(other.slots_); }

 private:
  struct Slot {
    Transform transform;
    bool      valid{false};
  };

  std::vector<Slot> slots_;
};

// The two most recently published steps, which bracket the instant the renderer
// wants to draw. curr_time is when curr was published.
struct PoseFrames {
  WorldSnapshot prev;
  WorldSnapshot curr;

  std::chrono::steady_clock::time_point curr_time{};

  // false until a second step has been published — nothing to blend from yet
  bool has_prev{false};
};

// Hands poses from the physics thread to the render thread.
//
// Physics accumulates a whole step into the back buffer and publishes it with an
// O(1) rotation, so a reader never observes a half-written step. Readers copy
// the published steps once per frame; physics therefore only ever blocks for the
// length of that copy, never for the length of a frame.
//
// Three buffers rather than two: interpolation needs the previous published step
// as well as the newest, and the retired buffer is recycled as the next back
// buffer so a steady-state step still allocates nothing.
class WorldBuffer {
 public:
  // physics thread only
  WorldSnapshot& writeBuffer() { return back_; }

  // stamp is when this step's state is current. It comes from the caller rather
  // than being read here, so a sim clock can drive it instead of wall time.
  void publish(std::chrono::steady_clock::time_point stamp);

  // any thread — overwrites out with the two most recently published steps
  void read(PoseFrames& out) const;

 private:
  mutable std::mutex mutex_;

  // guarded by mutex_
  WorldSnapshot                         prev_;
  WorldSnapshot                         curr_;
  std::chrono::steady_clock::time_point curr_time_{};
  bool                                  has_prev_{false};

  WorldSnapshot back_;  // physics thread only

  bool has_published_{false};
};

#endif
