#include <doctest.h>

#include <volasim/args.h>

#include <vector>

namespace {

// parseArgs takes argv as the C runtime hands it over, so the literals have to
// shed their constness to get there.
Args parse(const std::vector<const char*>& args) {
  std::vector<char*> argv;
  argv.reserve(args.size());
  for (const char* arg : args) {
    argv.push_back(const_cast<char*>(arg));
  }

  return parseArgs(static_cast<int>(argv.size()), argv.data());
}

}  // namespace

TEST_CASE("defaults are used when nothing is passed") {
  const Args args = parse({"volasim"});

  CHECK(args.physics_hz == doctest::Approx(1000.));
  CHECK_FALSE(args.report_rates);
  CHECK_FALSE(args.interpolate);
  CHECK_FALSE(args.world_path.empty());
}

TEST_CASE("flags are read") {
  const Args args =
      parse({"volasim", "--physics-hz", "250", "--rates", "--interpolate"});

  CHECK(args.physics_hz == doctest::Approx(250.));
  CHECK(args.report_rates);
  CHECK(args.interpolate);
}

TEST_CASE("--world sets the world path") {
  CHECK(parse({"volasim", "--world", "a/b.xml"}).world_path == "a/b.xml");
  CHECK(parse({"volasim", "-w", "a/b.xml"}).world_path == "a/b.xml");
  CHECK_THROWS_AS(parse({"volasim", "--world"}), std::runtime_error);
}

// Each of these once got through and reached the physics loop: inf and 1e300
// truncated the step to zero and the loop free-ran, 1e-30 overflowed the step
// duration and hung shutdown, and nan passed the bounds check outright.
TEST_CASE("degenerate physics rates are rejected") {
  for (const char* rate :
       {"inf", "-inf", "nan", "1e300", "1e-30", "0", "-5", "2e6"}) {
    CAPTURE(rate);
    CHECK_THROWS_AS(parse({"volasim", "--physics-hz", rate}),
                    std::runtime_error);
  }
}

TEST_CASE("malformed physics rates are rejected") {
  CHECK_THROWS_AS(parse({"volasim", "--physics-hz", "abc"}),
                  std::runtime_error);
  CHECK_THROWS_AS(parse({"volasim", "--physics-hz"}), std::runtime_error);
}

TEST_CASE("physics rates at the bounds are accepted") {
  CHECK(parse({"volasim", "--physics-hz", "0.001"}).physics_hz ==
        doctest::Approx(kMinPhysicsHz));
  CHECK(parse({"volasim", "--physics-hz", "1000000"}).physics_hz ==
        doctest::Approx(kMaxPhysicsHz));
}
