#ifndef WORLD_BUFFER_H
#define WORLD_BUFFER_H

#include <volasim/simulation/entity.h>
#include <volasim/simulation/transform.h>

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

// Double buffer handing poses from the physics thread to the render thread.
//
// Physics accumulates a whole step into the back buffer and publishes it with an
// O(1) swap, so a reader never observes a half-written step. Readers copy the
// published buffer once per frame; physics therefore only ever blocks for the
// length of that copy, never for the length of a frame.
class WorldBuffer {
 public:
  // physics thread only
  WorldSnapshot& writeBuffer() { return back_; }
  void           publish();

  // any thread — overwrites out with the most recently published step
  void read(WorldSnapshot& out) const;

 private:
  mutable std::mutex mutex_;

  WorldSnapshot front_;  // guarded by mutex_
  WorldSnapshot back_;   // physics thread only
};

#endif
