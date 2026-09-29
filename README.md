<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="assets/volasim-dark.svg">
    <img src="assets/volasim-light.svg" alt="volasim" width="520">
  </picture>
</p>

<p align="center">
  <a href="https://github.com/nocholasrift/volasim/actions/workflows/ci.yml"><img src="https://github.com/nocholasrift/volasim/actions/workflows/ci.yml/badge.svg" alt="CI"></a>
  <img src="https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white" alt="C++17">
  <img src="https://img.shields.io/badge/platform-Linux%20%7C%20macOS-lightgrey" alt="Platforms">
  <img src="https://img.shields.io/badge/ROS-1%20%7C%202-22314E?logo=ros&logoColor=white" alt="ROS 1 | ROS 2">
  <img src="https://img.shields.io/badge/physics-Jolt-orange" alt="Jolt Physics">
</p>

<p align="center">
  <b>A real-time quadrotor simulator with GPU depth sensors, a geometric controller, and a ZMQ bridge to ROS 1 and ROS 2.</b>
</p>

<!--
  DEMO: drop a GIF or MP4 here once recorded, e.g.
  <p align="center"><img src="assets/demo.gif" alt="volasim demo" width="800"></p>
-->

## 🔍 Overview

volasim renders a physics-driven world in OpenGL and streams the vehicle's state, TF tree, and depth point clouds over ZMQ. Controllers and planners run as separate processes. You can use the bundled standalone Lee controller, or the ROS bridge if you want to fly the drone from your existing ROS stack. 

## 🎯 Key Features

- 🚁 **Physics**
  - Rigid-body dynamics on [Jolt Physics](https://github.com/jrouwe/JoltPhysics), with collision geometry from convex decomposition
  - A physics loop that runs independently of rendering (`--physics-hz`, 1 kHz by default)
  - Optional interpolation between physics steps (`--interpolate`) for smooth motion at low physics rates
- 📷 **Sensors**
  - GPU-rendered depth cameras published as point clouds, with configurable resolution, FOV, range, and rate
  - An included RealSense D435i preset (`definitions/sensors/realsense.xml`)
  - A per-drone TF tree: `drone_N/odom → drone_N/base_link → drone_N/<sensor>`
- 🎮 **Control**
  - A geometric Lee (SE(3)) controller with feedforward from acceleration and jerk
  - A minimum-jerk trajectory generator for point-to-point commands
  - Tracking of full multi-DOF trajectories (`trajectory_msgs/MultiDOFJointTrajectory`)
- 🔌 **Comms**
  - Over ZMQ + protobuf, using [volasim-msgs](https://github.com/nocholasrift/volasim-msgs) for the message schema
  - A bridge that works with both ROS 1 and ROS 2; the build detects which one you have
  - A Dockerized ROS 2 stack, so the ROS side can run without ROS installed on the host
- 🗺️ **Worlds & Visualization**
  - XML world definitions with reusable classes, includes, and parameterized templates
  - OBJ meshes via [assimp](https://github.com/assimp/assimp), plus shadows, orbit camera, and live trajectory overlays

## 🚀 Getting Started

### Prerequisites

- CMake ≥ 3.16 and a C++17 compiler
- [vcpkg](https://github.com/microsoft/vcpkg) (supplies protobuf, abseil, glm, and eigen)
- OpenGL + GLUT
  - **Ubuntu:** `sudo apt install freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev libasound2-dev libudev-dev`
  - **macOS:** these ship with the system

SDL3, Jolt, [assimp](https://github.com/assimp/assimp), pugixml, and libzmq are vendored as submodules.

### Build

```bash
git clone --recursive https://github.com/nocholasrift/volasim.git
cd volasim
cmake -B build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build -j
```

### Fly (standalone, no ROS)

```bash
./build/run_standalone.sh
```

This launches the simulator and the ZMQ Lee controller, then commands the drone to a waypoint. To send it somewhere else:

```bash
./build/Release/send_position <x> <y> <z>
```

### Fly (ROS 2)

```bash
./scripts/run_ros2.sh     # sim + bridge + controller + position commander, then takes off
ros2 topic pub --once /command_pos geometry_msgs/msg/Point "{x: 1.0, y: 2.0, z: 2.0}"
```

`scripts/run_ros1.sh` does the same for ROS 1. If you'd rather keep ROS in a container, see [`docker/README.md`](docker/README.md).

> [!NOTE]
> Run the simulator from the repository root, because asset paths in world files are relative.

## 🧭 Usage

```text
volasim [--world <file.xml>] [--physics-hz <rate>] [--interpolate] [--rates]
```

| Flag | Default | Description |
| --- | --- | --- |
| `-w`, `--world` | `definitions/worlds/world_250_world.xml` | World definition to load |
| `--physics-hz` | `1000` | Physics step rate, independent of render FPS |
| `--interpolate` | off | Blend rendered poses between physics steps |
| `--rates` | off | Print the rate each loop actually achieves, once per second |

**Controls:** right-drag to orbit, scroll to zoom, `Q` to quit.

## 🔌 Interfaces

| Endpoint | Direction | Payload |
| --- | --- | --- |
| `tcp://*:5556` · `ipc:///tmp/volasim_state` | sim → out | Vehicle state, `tf`, `tf_static` |
| `tcp://*:5559` · `ipc:///tmp/volasim_cloud` | sim → out | Depth point clouds |
| `tcp://localhost:5557` | in → sim | Thrust + body torque command |
| `tcp://localhost:5560` · `ipc:///tmp/volasim_traj` | in → sim | Trajectory overlay for visualization |

Through the ROS bridge:

| Topic / Service | Type | |
| --- | --- | --- |
| `/odometry` | `nav_msgs/Odometry` | Vehicle state |
| `/command_pos` | `geometry_msgs/Point` | Go-to setpoint (min-jerk) |
| `/cmd_trajectory` | `trajectory_msgs/MultiDOFJointTrajectory` | Full trajectory to track |
| `/command` | `std_msgs/Float32MultiArray` | Controller output to the sim |
| `/takeoff`, `/land` | `std_srvs/Empty` | |

## ⚙️ World Configuration

Worlds are XML files in [`definitions/worlds/`](definitions/worlds). A vehicle carries its inertial parameters and can include sensors from templates:

```xml
<vehicle name="drone1" class="hummingbird">
  <mass>4.34</mass>
  <inertia_matrix>0.0820 0. 0. 0. 0.0845 0. 0. 0. .1377</inertia_matrix>
  <length>0.315</length>
  <c_torque>8.004e-4</c_torque>
  <init_pose>-2.25 2.5 0.5</init_pose>
  <include file="./definitions/sensors/realsense.xml" x="0.025" y="0.025" z="0.01" yaw="45"/>
</vehicle>
```

Collision meshes can be generated with `scripts/convex_decompose.py`.

## 🏗️ Architecture

```text
 ┌────────────── volasim ──────────────┐          ┌──────────────────────┐
 │  render thread  ◀── WorldBuffer ──┐ │  state   │  Lee controller      │
 │  (OpenGL, SDL3)     (3 snapshots) │ │ ───────▶ │  (standalone or ROS) │
 │                                   │ │          │                      │
 │  physics thread (Jolt) ───────────┘ │ ◀─────── │                      │
 │                                     │  thrust  └──────────────────────┘
 │  comms thread (ZMQ) ◀──────────────▶│ ───────▶  ROS bridge ─▶ /odometry, /tf,
 └─────────────────────────────────────┘  clouds                 point clouds
```

The render, physics, and comms threads each run at their own rate. Controllers get exact, un-interpolated state.

## 🧪 Tests

```bash
cmake --build build --target volasim_tests && ./build/Release/volasim_tests
python3 tests/mutation_check.py   # checks that the tests fail when the code they cover is broken
```

## 🗺️ Roadmap

- [ ] Demo videos
- [ ] Instanced trajectory rendering for dense trajectories
- [ ] Overlay frames resolved against the published TF tree
