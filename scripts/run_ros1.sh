#!/usr/bin/env bash
set -e

# === START PROCESSES ===

# Start Volasim (non-ROS binary)
echo "[INFO] Launching Volasim simulator..."
./build/Release/volasim &
VOLASIM_PID=$!

# start ros bridge
echo "[INFO] Starting ros bridge..."
./build/Release/vola_ros_bridge &
BRIDGE_PID=$!

# Start controller node
echo "[INFO] Launching Lee controller..."
./build/Release/lee_control_node &
CTRL_PID=$!

# Start position commander
echo "[INFO] Launching position commander..."
./build/Release/position_command_node &
CTRL_PID=$!

# sleep 5
timeout 30s rostopic echo -n1 /odometry >/dev/null
rosservice call /takeoff "{}"

# === CLEANUP HANDLER ===
trap "echo; echo '[INFO] Shutting down...'; kill $VOLASIM_PID $BRIDGE_PID $CTRL_PID; wait" SIGINT SIGTERM

# === KEEP SCRIPT ALIVE UNTIL CTRL+C ===
wait

