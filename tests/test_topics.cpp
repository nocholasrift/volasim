#include <doctest.h>

#include <volasim/comms/frame_registry.h>
#include <volasim/comms/frames.h>
#include <volasim/comms/topics.h>

using namespace volasim;

TEST_CASE("topic strings follow the drone/<id>/<stream> contract") {
  CHECK(topics::drone(7) == "drone/7");
  CHECK(topics::state(7) == "drone/7/state");
  CHECK(topics::depth(7, "front") == "drone/7/front/depth");
  CHECK(topics::tf(7) == "drone/7/tf");
  CHECK(topics::tfStatic(7) == "drone/7/tf_static");
}

TEST_CASE("two sensors on one drone get distinct depth topics") {
  // The whole point of naming the sensor into the topic: two sensors must not
  // share a stream, or a single downstream cloud topic flickers between them.
  CHECK(topics::depth(7, "front") != topics::depth(7, "rear"));
}

TEST_CASE("each drone gets its own topic namespace") {
  CHECK(topics::state(0) != topics::state(1));
  CHECK(topics::depth(0, "cam") != topics::depth(1, "cam"));
  CHECK(topics::tf(0) != topics::tf(1));
  CHECK(topics::tfStatic(0) != topics::tfStatic(1));
}

TEST_CASE("a per-drone prefix subscription matches that drone's streams") {
  // SUB-side prefix filtering keys off the leading frame, so the drone prefix
  // must be a prefix of every stream for that drone and of no other drone's.
  const std::string prefix = topics::drone(3) + "/";

  CHECK(topics::state(3).rfind(prefix, 0) == 0);
  CHECK(topics::depth(3, "cam").rfind(prefix, 0) == 0);
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

TEST_CASE("frame registry keeps a sensor off the reserved root frames") {
  frames::Registry reg;
  reg.reserveRoots(0);

  const auto base = reg.assignSensor(0, "base_link");
  CHECK(base.link != frames::baseLink(0));
  CHECK(base.link == "drone_0/base_link_1");

  const auto odom = reg.assignSensor(0, "odom");
  CHECK(odom.link != frames::odom(0));
  CHECK(odom.link == "drone_0/odom_1");
}

TEST_CASE("frame registry disambiguates a name that hits another sensor's optical frame") {
  frames::Registry reg;

  const auto a = reg.assignSensor(0, "foo");
  CHECK(a.link == "drone_0/foo");
  CHECK(a.optical == "drone_0/foo_optical");

  // "foo_optical" would produce link drone_0/foo_optical, already taken by a's
  // optical frame, so it must be bumped.
  const auto b = reg.assignSensor(0, "foo_optical");
  CHECK(b.link != a.optical);
  CHECK(b.link == "drone_0/foo_optical_1");
}

TEST_CASE("frame registry gives duplicate sensor names distinct frames") {
  frames::Registry reg;
  const auto a = reg.assignSensor(0, "cam");
  const auto b = reg.assignSensor(0, "cam");
  CHECK(a.link != b.link);
  CHECK(a.optical != b.optical);
  CHECK(b.link == "drone_0/cam_1");
}

TEST_CASE("registry folds names illegal as ROS tokens into legal ones") {
  // A hyphen (or any char outside [A-Za-z0-9_]) is rejected by ROS topic
  // naming; the registry must fold it so the derived depth topic is valid.
  frames::Registry reg;
  const auto s = reg.assignSensor(0, "front-depth");
  CHECK(s.name == "front_depth");
  CHECK(s.link == "drone_0/front_depth");
  CHECK(topics::depth(0, s.name) == "drone/0/front_depth/depth");
}

TEST_CASE("folded names that would alias are bumped apart, not merged") {
  // "front-depth" and "front_depth" fold to the same token, so routing would be
  // ambiguous if they shared it; the second must be disambiguated.
  frames::Registry reg;
  const auto a = reg.assignSensor(0, "front-depth");
  const auto b = reg.assignSensor(0, "front_depth");
  CHECK(a.name == "front_depth");
  CHECK(b.name == "front_depth_1");
  CHECK(topics::depth(0, a.name) != topics::depth(0, b.name));
}

TEST_CASE("registry reports the (possibly bumped) name the frames were built from") {
  // The depth topic is built from this name, so it must track the bump that
  // disambiguated the frames — otherwise a bumped sensor's topic and frame
  // would disagree.
  frames::Registry reg;
  const auto a = reg.assignSensor(0, "cam");
  const auto b = reg.assignSensor(0, "cam");
  CHECK(a.name == "cam");
  CHECK(b.name == "cam_1");
  CHECK(a.link == frames::sensor(0, a.name));
  CHECK(topics::depth(0, b.name) == "drone/0/cam_1/depth");
}

TEST_CASE("frame registry disambiguates per drone independently") {
  frames::Registry reg;
  const auto a = reg.assignSensor(0, "cam");
  const auto b = reg.assignSensor(1, "cam");
  CHECK(a.link == "drone_0/cam");
  CHECK(b.link == "drone_1/cam");  // different drone: no suffix needed
}
