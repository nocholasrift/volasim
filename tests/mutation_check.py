#!/usr/bin/env python3
"""Checks that the unit tests can actually fail.

Each entry below breaks one behaviour the suite claims to protect, rebuilds, and
expects the tests to go red. A mutation reported as MISSED means the tests pass
with the behaviour broken, so they are not testing what they look like they are.

Usage: python3 tests/mutation_check.py   (from the repo root, after configuring build/)
"""

import subprocess
import sys

# (description, file, code to break, what to break it into)
MUTATIONS = [
    (
        "shortest-path rotation blend",
        "include/volasim/simulation/transform.h",
        "glm::normalize(glm::slerp(a.rotation, b.rotation, alpha))",
        "glm::normalize(glm::lerp(a.rotation, b.rotation, alpha))",
    ),
    (
        "blendFrom actually blends",
        "src/simulation/world_buffer.cpp",
        "    slots_[id].transform   = start != nullptr\n"
        "                                 ? lerp(*start, target.transform, alpha)\n"
        "                                 : target.transform;",
        "    slots_[id].transform   = target.transform;",
    ),
    (
        "recycled buffer is cleared",
        "src/simulation/world_buffer.cpp",
        "  back_.invalidate();",
        "  // back_.invalidate();",
    ),
    (
        "snapshot invalidation",
        "src/simulation/world_buffer.cpp",
        "  for (Slot& slot : slots_) {\n    slot.valid = false;\n  }",
        "  // no-op",
    ),
    (
        "previous step is retained on publish",
        "src/simulation/world_buffer.cpp",
        "    prev_.swap(curr_);\n    curr_.swap(back_);",
        "    curr_.swap(back_);",
    ),
    (
        "interpolation alpha is clamped",
        "src/simulation/world_buffer.cpp",
        "std::clamp(elapsed / step_seconds, 0., 1.)",
        "(elapsed / step_seconds)",
    ),
    (
        "pacer keeps a fixed schedule",
        "include/volasim/simulation/loop_pacer.h",
        "    next_ += step_;",
        "    next_ = now + step_;",
    ),
    (
        "pacer drops missed steps",
        "include/volasim/simulation/loop_pacer.h",
        "    if (next_ < now) {\n"
        "      dropped_ += static_cast<unsigned int>((now - next_) / step_) + 1;\n"
        "      next_     = now + step_;\n"
        "    }\n\n",
        "",
    ),
    (
        "physics rate bounds",
        "include/volasim/args.h",
        "      if (!std::isfinite(args.physics_hz) || args.physics_hz < kMinPhysicsHz ||\n"
        "          args.physics_hz > kMaxPhysicsHz) {",
        "      if (args.physics_hz <= 0.) {",
    ),
]


FLAGS_FILE = "build/tests/CMakeFiles/volasim_tests.dir/flags.make"
BINARY = "build/volasim_tests_mutation"

SOURCES = [
    "tests/main.cpp",
    "tests/test_args.cpp",
    "tests/test_transform.cpp",
    "tests/test_world_buffer.cpp",
    "tests/test_loop_pacer.cpp",
    "src/simulation/world_buffer.cpp",
]


def compile_flags():
    """Flags CMake uses for the test target, so this compiles what CMake would.

    Requires build/ to have been configured at least once.
    """
    defines, includes, flags = "", "", ""
    with open(FLAGS_FILE) as handle:
        for line in handle:
            if line.startswith("CXX_DEFINES ="):
                defines = line.split("=", 1)[1]
            elif line.startswith("CXX_INCLUDES ="):
                includes = line.split("=", 1)[1]
            elif line.startswith("CXX_FLAGS ="):
                flags = line.split("=", 1)[1]

    return (defines + " " + includes + " " + flags).split()


def write(path, text):
    with open(path, "w") as handle:
        handle.write(text)


def build_and_run(flags):
    """Compiles the suite in one shot and runs it.

    Deliberately not an incremental build: mutating a header and rebuilding
    left the odd object holding the previous edit, and a mutation judged
    against a stale binary reports whatever the last build happened to say.
    Compiling every translation unit each time costs a few seconds and removes
    the question entirely.
    """
    build = subprocess.run(
        ["c++"] + flags + SOURCES + ["-o", BINARY], capture_output=True, text=True
    )
    if build.returncode != 0:
        return None

    return subprocess.run([BINARY], capture_output=True, text=True)


def main():
    # With a red baseline every mutation looks caught, because the tests were
    # already failing before anything was broken.
    flags = compile_flags()

    baseline = build_and_run(flags)
    if baseline is None:
        print("baseline does not build; fix that before mutating")
        return 1
    if baseline.returncode != 0:
        print("baseline tests already fail; fix that before mutating")
        return 1

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
            result = build_and_run(flags)
        finally:
            write(path, original)

        if result is None:
            # the compiler rejected the break, which protects the behaviour too
            print(f"  caught   {name} (does not compile when broken)")
        elif result.returncode != 0:
            print(f"  caught   {name}")
        else:
            print(f"  MISSED   {name}")
            missed.append(name)

    # A restore that did not take would leave the tree broken while every
    # mutation above still reported a tidy result.
    restored = build_and_run(flags)
    if restored is None or restored.returncode != 0:
        print("\ntree is not clean after restoring; check for a failed restore")
        if restored is not None:
            print(restored.stdout[-2000:])
        return 1

    if missed:
        print(f"\n{len(missed)} mutation(s) not caught: {', '.join(missed)}")
        return 1

    print(f"\nall {len(MUTATIONS)} mutations caught")
    return 0


if __name__ == "__main__":
    sys.exit(main())
