#include <doctest.h>

#include <volasim/comms/topics.h>

using namespace volasim;

TEST_CASE("topic strings follow the drone/<id>/<stream> contract") {
  CHECK(topics::drone(7) == "drone/7");
  CHECK(topics::state(7) == "drone/7/state");
  CHECK(topics::depth(7) == "drone/7/depth");
}

TEST_CASE("each drone gets its own topic namespace") {
  CHECK(topics::state(0) != topics::state(1));
  CHECK(topics::depth(0) != topics::depth(1));
}

TEST_CASE("a per-drone prefix subscription matches that drone's streams") {
  // SUB-side prefix filtering keys off the leading frame, so the drone prefix
  // must be a prefix of every stream for that drone and of no other drone's.
  const std::string prefix = topics::drone(3) + "/";

  CHECK(topics::state(3).rfind(prefix, 0) == 0);
  CHECK(topics::depth(3).rfind(prefix, 0) == 0);
  CHECK(topics::state(30).rfind(prefix, 0) != 0);  // "drone/30" must not match "drone/3/"
}
