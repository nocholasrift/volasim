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

void WorldSnapshot::blendFrom(const WorldSnapshot& prev,
                              const WorldSnapshot& curr, float alpha) {
  slots_.resize(curr.slots_.size());

  for (std::size_t id = 0; id < curr.slots_.size(); ++id) {
    const Slot& target = curr.slots_[id];

    slots_[id].valid = target.valid;
    if (!target.valid) {
      continue;
    }

    const Transform* start = prev.find(static_cast<EntityID>(id));
    slots_[id].transform   = start != nullptr
                                 ? lerp(*start, target.transform, alpha)
                                 : target.transform;
  }
}

void WorldSnapshot::invalidate() {
  for (Slot& slot : slots_) {
    slot.valid = false;
  }
}

void WorldBuffer::publish(std::chrono::steady_clock::time_point stamp) {
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // rotate: curr retires to prev, the finished step becomes curr, and the
    // buffer prev just gave up is recycled as the next back buffer
    prev_.swap(curr_);
    curr_.swap(back_);

    curr_time_ = stamp;
    has_prev_  = has_published_;
  }

  has_published_ = true;

  // back_ now holds the step before last; the next step refills it from scratch
  back_.invalidate();
}

void WorldBuffer::read(PoseFrames& out) const {
  std::lock_guard<std::mutex> lock(mutex_);

  out.prev      = prev_;
  out.curr      = curr_;
  out.curr_time = curr_time_;
  out.has_prev  = has_prev_;
}
