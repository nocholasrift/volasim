#include <doctest.h>

#include <volasim/simulation/transform.h>

#include <glm/gtc/constants.hpp>

namespace {

// Two quaternions describe the same rotation when they are parallel, sign
// included: q and -q rotate identically.
bool sameRotation(const glm::quat& a, const glm::quat& b) {
  return std::abs(glm::dot(a, b)) > 0.9999F;
}

glm::quat aboutZ(float degrees) {
  return glm::angleAxis(glm::radians(degrees), glm::vec3(0.F, 0.F, 1.F));
}

}  // namespace

TEST_CASE("lerp returns its endpoints") {
  Transform a;
  a.position = {1.F, 2.F, 3.F};
  a.rotation = aboutZ(20.F);
  a.scale    = {1.F, 1.F, 1.F};

  Transform b;
  b.position = {4.F, 6.F, 8.F};
  b.rotation = aboutZ(80.F);
  b.scale    = {2.F, 2.F, 2.F};

  const Transform at_start = lerp(a, b, 0.F);
  const Transform at_end   = lerp(a, b, 1.F);

  CHECK(at_start.position.x == doctest::Approx(a.position.x));
  CHECK(at_start.position.z == doctest::Approx(a.position.z));
  CHECK(sameRotation(at_start.rotation, a.rotation));

  CHECK(at_end.position.x == doctest::Approx(b.position.x));
  CHECK(at_end.position.z == doctest::Approx(b.position.z));
  CHECK(sameRotation(at_end.rotation, b.rotation));
}

TEST_CASE("lerp splits position and scale evenly") {
  Transform a;
  a.position = {0.F, 0.F, 0.F};
  a.scale    = {1.F, 1.F, 1.F};

  Transform b;
  b.position = {10.F, -4.F, 2.F};
  b.scale    = {3.F, 3.F, 3.F};

  const Transform mid = lerp(a, b, 0.5F);

  CHECK(mid.position.x == doctest::Approx(5.F));
  CHECK(mid.position.y == doctest::Approx(-2.F));
  CHECK(mid.position.z == doctest::Approx(1.F));
  CHECK(mid.scale.x == doctest::Approx(2.F));
}

// A quaternion and its negation are the same rotation but blend in opposite
// directions. Without the shortest-path flip this halfway point comes out at
// -135 degrees instead of 45 — the long way round.
TEST_CASE("lerp takes the short way round a negated quaternion") {
  Transform a;
  a.rotation = glm::quat(1.F, 0.F, 0.F, 0.F);

  Transform b;
  b.rotation = -aboutZ(90.F);

  const Transform mid = lerp(a, b, 0.5F);

  CHECK(sameRotation(mid.rotation, aboutZ(45.F)));
  CHECK_FALSE(sameRotation(mid.rotation, aboutZ(-135.F)));
}

TEST_CASE("lerp leaves the rotation normalized") {
  Transform a;
  a.rotation = aboutZ(10.F);

  Transform b;
  b.rotation = aboutZ(170.F);

  for (float alpha : {0.F, 0.25F, 0.5F, 0.75F, 1.F}) {
    CAPTURE(alpha);
    CHECK(glm::length(lerp(a, b, alpha).rotation) == doctest::Approx(1.F));
  }
}
