# Dockerized ROS2 side

Runs the **ROS2 half** of volasim — the ZMQ↔ROS2 bridge and the Lee
controller — in one container. The **simulator stays on the host** (it needs the
GPU/OpenGL window), and the two sides talk over ZMQ.

```
        HOST                                CONTAINER (this image)
  ┌────────────────┐                    ┌──────────────────────────────┐
  │ volasim         │                   │  vola_ros_bridge              │
  │  PUB  *:5556 ───┼─── state ────────▶│   SUB connect :5556           │
  │  SUB  :5557 ◀───┼─── thrust ────────┤   PUB  bind  *:5557           │
  └────────────────┘                    │        ▲   ROS2 topics/DDS    │
                                         │        ▼   (never leaves box) │
                                         │  lee_control_node             │
                                         └──────────────────────────────┘
```

- **State (5556):** sim binds on the host; the bridge connects out. The
  container reaches the host via `VOLASIM_SIM_HOST` (`host.docker.internal`).
- **Thrust (5557):** the bridge binds; the sim connects to `localhost:5557`.
  Compose publishes the port so the host's `localhost:5557` forwards in.
- **bridge ↔ controller:** ordinary ROS2 topics over DDS, entirely inside the
  container. Nothing DDS-related is exposed to the host.

## Usage

```bash
# from the repo root
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up        # start bridge + controller
./build/volasim                                        # start the sim on the host
```

Auto-takeoff on startup (mirrors `scripts/run_ros2.sh`):

```bash
AUTO_TAKEOFF=1 docker compose -f docker/docker-compose.yml up
```

### Launcher script

`docker/run.sh` wraps `docker compose up --build` with convenient flags:

```bash
./docker/run.sh              # start bridge + controller
./docker/run.sh --record     # ...and record an MCAP bag
./docker/run.sh -r -t        # record + auto-takeoff
```

### Recording bags

`RECORD=1` records `/odometry` (pose **and** velocity), `/command_pos` (setpoint),
and `/command` (control output) as an MCAP bag:

```bash
RECORD=1 docker compose -f docker/docker-compose.yml up
```

Bags land in `<repo>/bags/run_<timestamp>/` (bind-mounted, so they persist on the
host). The bag is finalized on shutdown — stop the stack with `Ctrl-C` (or
`docker compose down`) rather than killing it, so the MCAP file closes cleanly.
Open the result in **Foxglove Studio** (native macOS app, drag-and-drop `.mcap`)
or PlotJuggler.

## Notes

- **Build reuse:** `docker/CMakeLists.txt` is a thin root shim that supplies
  `myproto`, `cppzmq`, and Eigen, then `add_subdirectory(volasim_node)` — the
  real node CMake is reused unchanged. The simulator's heavy deps
  (SDL/Jolt/assimp/OpenGL) are never built here.
- **The Python trajectory script is not included** — `scripts/publish_trajectory.py`
  uses `rospy` (ROS1). It needs an `rclpy` port before it can run in this image.
- **Adding more ROS2 nodes:** run them in this container (or add compose services
  sharing the default network and a common `ROS_DOMAIN_ID`) so DDS discovery
  works without exposing DDS to the host.
- **Linux vs Docker Desktop:** `extra_hosts: host-gateway` makes
  `host.docker.internal` resolve on Linux; on macOS/Windows Docker Desktop it is
  already provided.

## Adding ROS1 later

The setup is deliberately structured so a ROS1 variant slots in alongside the
ROS2 one — most pieces are already shared:

| Piece | Shared? | Notes |
|-------|---------|-------|
| `docker/CMakeLists.txt` | ✅ reuse as-is | `volasim_node/CMakeLists.txt` already auto-detects catkin (ROS1) vs rclcpp (ROS2). |
| `docker/entrypoint.sh` | ✅ reuse as-is | Sources `/opt/ros/$ROS_DISTRO/...`; the base image sets `ROS_DISTRO`. |
| `Dockerfile.ros1` | 🆕 new sibling | `FROM ros:noetic`; swap `ros-humble-*` apt packages for `ros-noetic-*`. Build steps are otherwise identical. |
| `run_ros1_stack.sh` | 🆕 new sibling | Start `roscore` first, then `vola_ros_bridge` + `lee_control_node` + `position_command_node`; use `rosservice call /takeoff`. |
| compose service | 🆕 new service/file | Add `ROS_MASTER_URI`/`ROS_HOSTNAME`; a `roscore` is needed (in-container). ZMQ ports (5556/5557) and `VOLASIM_SIM_HOST` are identical. |

The ROS1 sources already exist (`vola_ros1_bridge.cpp`, `lee_control_ros1.cpp`,
`position_commander_ros1.cpp`) and the node CMake's ROS1 branch already builds
them — so no C++ changes are needed, only the container plumbing above.
