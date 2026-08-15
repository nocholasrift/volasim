#include <doctest.h>

#include <volasim/simulation/world_buffer.h>

namespace {

using Clock = std::chrono::steady_clock;

Transform poseAt(float z) {
  Transform pose;
  pose.position = {0.F, 0.F, z};
  return pose;
}

Clock::time_point at(double seconds) {
  return Clock::time_point{} + std::chrono::duration_cast<Clock::duration>(
                                   std::chrono::duration<double>(seconds));
}

}  // namespace

TEST_CASE("a snapshot returns only the poses it was given") {
  WorldSnapshot snapshot;
  snapshot.set(7, poseAt(3.F));

  REQUIRE(snapshot.find(7) != nullptr);
  CHECK(snapshot.find(7)->position.z == doctest::Approx(3.F));

  CHECK(snapshot.find(6) == nullptr);     // never set
  CHECK(snapshot.find(9999) == nullptr);  // beyond anything stored

  snapshot.invalidate();
  CHECK(snapshot.find(7) == nullptr);
}

TEST_CASE("blendFrom interpolates entities present in both") {
  WorldSnapshot prev;
  WorldSnapshot curr;
  prev.set(1, poseAt(0.F));
  curr.set(1, poseAt(10.F));

  WorldSnapshot blended;
  blended.blendFrom(prev, curr, 0.25F);

  REQUIRE(blended.find(1) != nullptr);
  CHECK(blended.find(1)->position.z == doctest::Approx(2.5F));
}

TEST_CASE("blendFrom takes new entities as they are and drops departed ones") {
  WorldSnapshot prev;
  WorldSnapshot curr;
  prev.set(1, poseAt(0.F));  // in prev only — gone this step
  curr.set(2, poseAt(8.F));  // in curr only — nothing to blend from

  WorldSnapshot blended;
  blended.blendFrom(prev, curr, 0.5F);

  CHECK(blended.find(1) == nullptr);
  REQUIRE(blended.find(2) != nullptr);
  CHECK(blended.find(2)->position.z == doctest::Approx(8.F));
}

TEST_CASE("a reader sees nothing to blend until two steps are published") {
  WorldBuffer buffer;
  PoseFrames  frames;

  buffer.read(frames);
  CHECK_FALSE(frames.has_prev);

  buffer.writeBuffer().set(1, poseAt(1.F));
  buffer.publish(at(1.));

  buffer.read(frames);
  CHECK_FALSE(frames.has_prev);  // only one step exists
  REQUIRE(frames.curr.find(1) != nullptr);
  CHECK(frames.curr.find(1)->position.z == doctest::Approx(1.F));

  buffer.writeBuffer().set(1, poseAt(2.F));
  buffer.publish(at(2.));

  buffer.read(frames);
  REQUIRE(frames.has_prev);
  CHECK(frames.prev.find(1)->position.z == doctest::Approx(1.F));
  CHECK(frames.curr.find(1)->position.z == doctest::Approx(2.F));
  CHECK(frames.curr_time == at(2.));
}

TEST_CASE("published steps stay one apart as the buffers rotate") {
  WorldBuffer buffer;
  PoseFrames  frames;

  for (int step = 1; step <= 6; ++step) {
    buffer.writeBuffer().set(1, poseAt(static_cast<float>(step)));
    buffer.publish(at(step));

    buffer.read(frames);
    REQUIRE(frames.curr.find(1) != nullptr);
    CHECK(frames.curr.find(1)->position.z == doctest::Approx(step));

    if (step > 1) {
      REQUIRE(frames.prev.find(1) != nullptr);
      CHECK(frames.prev.find(1)->position.z == doctest::Approx(step - 1));
    }
  }
}

// The buffer a step is written into was published earlier and still holds that
// step's entities, so it has to be cleared before the renderer sees it again.
// A buffer takes four publishes to come back round as curr, which is how long
// this has to run to catch a missing clear.
TEST_CASE("a recycled buffer carries no stale entities") {
  WorldBuffer buffer;
  PoseFrames  frames;

  // entity 1 exists for this step only
  buffer.writeBuffer().set(1, poseAt(1.F));
  buffer.writeBuffer().set(2, poseAt(1.F));
  buffer.publish(at(1.));

  for (int step = 2; step <= 4; ++step) {
    buffer.writeBuffer().set(2, poseAt(static_cast<float>(step)));
    buffer.publish(at(step));

    buffer.read(frames);
    CAPTURE(step);
    CHECK(frames.curr.find(1) == nullptr);
    REQUIRE(frames.curr.find(2) != nullptr);
    CHECK(frames.curr.find(2)->position.z == doctest::Approx(step));
  }
}

TEST_CASE("interpolation alpha spans exactly one step") {
  const Clock::time_point published = at(10.);
  const double            step      = 0.01;  // 100 Hz

  CHECK(interpolationAlpha(published, published, step) == doctest::Approx(0.F));
  CHECK(interpolationAlpha(at(10.005), published, step) ==
        doctest::Approx(0.5F));
  CHECK(interpolationAlpha(at(10.01), published, step) == doctest::Approx(1.F));
}

// Past the newest step there is no state to blend towards, so the alpha holds
// instead of running on into poses physics never computed.
TEST_CASE("interpolation alpha is clamped at both ends") {
  const Clock::time_point published = at(10.);
  const double            step      = 0.01;

  CHECK(interpolationAlpha(at(10.05), published, step) == doctest::Approx(1.F));
  CHECK(interpolationAlpha(at(9.9), published, step) == doctest::Approx(0.F));
}
