#include <volasim/simulation/world_buffer.h>

void WorldSnapshot::set(EntityID id, const Transform& transform) {
  if (id >= slots_.size()) {
    slots_.resize(id + 1);
  }

  slots_[id].transform = transform;
  slots_[id].valid     = true;
}

const Transform* WorldSnapshot::find(EntityID id) const {
  if (id >= slots_.size() || !slots_[id].valid) {
    return nullptr;
  }

  return &slots_[id].transform;
}

void WorldSnapshot::invalidate() {
  for (Slot& slot : slots_) {
    slot.valid = false;
  }
}

void WorldBuffer::publish() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    front_.swap(back_);
  }

  // back_ now holds the step before last; the next step refills it from scratch
  back_.invalidate();
}

void WorldBuffer::read(WorldSnapshot& out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  out = front_;
}
