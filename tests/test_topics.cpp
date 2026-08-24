#include <doctest.h>

#include <volasim/comms/frames.h>
#include <volasim/comms/topics.h>

using namespace volasim;

TEST_CASE("topic strings follow the drone/<id>/<stream> contract") {
  CHECK(topics::drone(7) == "drone/7");
  CHECK(topics::state(7) == "drone/7/state");
  CHECK(topics::depth(7) == "drone/7/depth");
  CHECK(topics::tf(7) == "drone/7/tf");
  CHECK(topics::tfStatic(7) == "drone/7/tf_static");
}

TEST_CASE("each drone gets its own topic namespace") {
  CHECK(topics::state(0) != topics::state(1));
  CHECK(topics::depth(0) != topics::depth(1));
  CHECK(topics::tf(0) != topics::tf(1));
  CHECK(topics::tfStatic(0) != topics::tfStatic(1));
}

TEST_CASE("a per-drone prefix subscription matches that drone's streams") {
  // SUB-side prefix filtering keys off the leading frame, so the drone prefix
  // must be a prefix of every stream for that drone and of no other drone's.
  const std::string prefix = topics::drone(3) + "/";

  CHECK(topics::state(3).rfind(prefix, 0) == 0);
  CHECK(topics::depth(3).rfind(prefix, 0) == 0);
  CHECK(topics::tf(3).rfind(prefix, 0) == 0);
  CHECK(topics::tfStatic(3).rfind(prefix, 0) == 0);
  CHECK(topics::state(30).rfind(prefix, 0) != 0);  // "drone/30" must not match "drone/3/"
}

TEST_CASE("tf is a prefix of tf_static — consumers must match the full topic") {
  // WARNING for future subscribers: "drone/<id>/tf" is a byte-prefix of
  // "drone/<id>/tf_static", so a ZMQ SUB *prefix* filter on tf also catches
  // tf_static. The bridge sidesteps this by subscribing with an empty filter and
  // routing on the exact topic string, never a prefix.
  CHECK(topics::tf(5) != topics::tfStatic(5));
  CHECK(topics::tfStatic(5).rfind(topics::tf(5), 0) == 0);
}

TEST_CASE("tf frame names namespace every frame under the drone") {
  CHECK(frames::drone(2) == "drone_2");
  CHECK(frames::odom(2) == "drone_2/odom");
  CHECK(frames::baseLink(2) == "drone_2/base_link");
  CHECK(frames::sensor(2, "front_depth") == "drone_2/front_depth");
}

TEST_CASE("a sensor's optical frame is a child of its link frame") {
  // The optical frame must extend the link frame's name so the two are visibly
  // paired and never collide with another sensor's frames.
  CHECK(frames::sensorOptical(2, "front_depth") ==
        "drone_2/front_depth_optical");
  CHECK(frames::sensorOptical(2, "front_depth") !=
        frames::sensor(2, "front_depth"));
  CHECK(frames::sensorOptical(2, "front_depth")
            .rfind(frames::sensor(2, "front_depth"), 0) == 0);
}

TEST_CASE("each drone's frames are disjoint from another's") {
  CHECK(frames::odom(0) != frames::odom(1));
  CHECK(frames::baseLink(0) != frames::baseLink(1));
  CHECK(frames::sensor(0, "depth") != frames::sensor(1, "depth"));
  CHECK(frames::sensor(0, "depth_a") != frames::sensor(0, "depth_b"));
}
