#!/usr/bin/env bash
set -e

# === SETUP ===
# Source ROS and your workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# === START PROCESSES ===

# Start Volasim (non-ROS binary)
echo "[INFO] Launching Volasim simulator..."
./build/Release/volasim &
VOLASIM_PID=$!

# start ros bridge
echo "[INFO] Starting ros bridge..."
./build/Release/vola_ros_bridge &
CTRL_PID=$!

# Start controller node
echo "[INFO] Launching Lee controller..."
./build/Release/lee_control_node &
CTRL_PID=$!

# === CLEANUP HANDLER ===
trap "echo; echo '[INFO] Shutting down...'; kill $VOLASIM_PID $CTRL_PID $RVIZ_PID; wait" SIGINT SIGTERM

# === KEEP SCRIPT ALIVE UNTIL CTRL+C ===
wait

