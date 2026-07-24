#!/usr/bin/env bash
# ROS2 stack: bridge + controller. Sim runs on the host.
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

/ws/build/vola_ros_bridge &
BRIDGE_PID=$!

/ws/build/lee_control_node &
CTRL_PID=$!

RECORD_PID=""
if [ "${RECORD:-0}" = "1" ]; then
  mkdir -p /ws/bags
  BAG_DIR="/ws/bags/run_$(date +%Y%m%d_%H%M%S)"
  echo "[run_ros2_stack] recording MCAP bag to ${BAG_DIR}"
  ros2 bag record -s mcap -o "${BAG_DIR}" /odometry /command_pos /command &
  RECORD_PID=$!
fi

shutdown() {
  echo "[run_ros2_stack] shutting down..."
  # SIGINT the recorder first and wait, so the MCAP file is finalized cleanly.
  if [ -n "${RECORD_PID}" ]; then
    kill -INT "${RECORD_PID}" 2>/dev/null || true
    wait "${RECORD_PID}" 2>/dev/null || true
  fi
  kill "$BRIDGE_PID" "$CTRL_PID" 2>/dev/null || true
  wait
}
trap shutdown SIGINT SIGTERM

if [ "${AUTO_TAKEOFF:-0}" = "1" ]; then
  sleep 2
  ros2 service call /takeoff std_srvs/srv/Empty {} || true
fi

wait -n  # exit if any process dies; restart policy handles it
shutdown
