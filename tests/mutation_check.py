#!/usr/bin/env python3
"""Checks that the unit tests can actually fail.

Each entry below breaks one behaviour the suite claims to protect, rebuilds, and
expects the tests to go red. A mutation reported as MISSED means the tests pass
with the behaviour broken, so they are not testing what they look like they are.

Usage: python3 tests/mutation_check.py   (from the repo root, after configuring build/)
"""

import os
import subprocess
import sys
import time

# (description, file, code to break, what to break it into)
MUTATIONS = [
    ("shortest-path rotation blend", "include/volasim/simulation/transform.h",
     "glm::normalize(glm::slerp(a.rotation, b.rotation, alpha))",
     "glm::normalize(glm::lerp(a.rotation, b.rotation, alpha))"),
    ("blendFrom actually blends", "src/simulation/world_buffer.cpp",
     "    slots_[id].transform   = start != nullptr\n"
     "                                 ? lerp(*start, target.transform, alpha)\n"
     "                                 : target.transform;",
     "    slots_[id].transform   = target.transform;"),
    ("recycled buffer is cleared", "src/simulation/world_buffer.cpp",
     "  back_.invalidate();", "  // back_.invalidate();"),
    ("snapshot invalidation", "src/simulation/world_buffer.cpp",
     "  for (Slot& slot : slots_) {\n    slot.valid = false;\n  }", "  // no-op"),
    ("previous step is retained on publish", "src/simulation/world_buffer.cpp",
     "    prev_.swap(curr_);\n    curr_.swap(back_);", "    curr_.swap(back_);"),
    ("interpolation alpha is clamped", "src/simulation/world_buffer.cpp",
     "std::clamp(elapsed / step_seconds, 0., 1.)", "(elapsed / step_seconds)"),
    ("pacer keeps a fixed schedule", "include/volasim/simulation/loop_pacer.h",
     "    next_ += step_;", "    next_ = now + step_;"),
    ("pacer drops missed steps", "include/volasim/simulation/loop_pacer.h",
     "    if (next_ < now) {\n"
     "      dropped_ += static_cast<unsigned int>((now - next_) / step_) + 1;\n"
     "      next_     = now + step_;\n"
     "    }\n\n", ""),
    ("physics rate bounds", "include/volasim/args.h",
     "      if (!std::isfinite(args.physics_hz) || args.physics_hz < kMinPhysicsHz ||\n"
     "          args.physics_hz > kMaxPhysicsHz) {",
     "      if (args.physics_hz <= 0.) {"),
]


def write(path, text):
    with open(path, "w") as handle:
        handle.write(text)
    # Make's mtime comparison is coarse enough that a rewrite landing in the same
    # instant as the last build gets treated as up to date, which silently tests
    # a stale binary. Push the mtime forward so the rebuild always happens.
    stamp = time.time() + 1
    os.utime(path, (stamp, stamp))


def build_and_run():
    build = subprocess.run(["cmake", "--build", "build", "--target", "volasim_tests", "-j8"],
                           capture_output=True, text=True)
    if build.returncode != 0:
        return None
    return subprocess.run(["./build/volasim_tests"], capture_output=True, text=True)


def main():
    missed = []

    for name, path, original_code, broken_code in MUTATIONS:
        with open(path) as handle:
            original = handle.read()

        if original_code not in original:
            print(f"  SKIPPED  {name}: code has moved, update this mutation")
            missed.append(name)
            continue

        write(path, original.replace(original_code, broken_code, 1))
        try:
            result = build_and_run()
        finally:
            write(path, original)

        if result is None:
            print(f"  SKIPPED  {name}: does not compile when broken")
        elif result.returncode != 0:
            print(f"  caught   {name}")
        else:
            print(f"  MISSED   {name}")
            missed.append(name)

    subprocess.run(["cmake", "--build", "build", "--target", "volasim_tests", "-j8"],
                   capture_output=True)

    if missed:
        print(f"\n{len(missed)} mutation(s) not caught: {', '.join(missed)}")
        return 1

    print(f"\nall {len(MUTATIONS)} mutations caught")
    return 0


if __name__ == "__main__":
    sys.exit(main())
