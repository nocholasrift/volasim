#!/usr/bin/env bash
# Run from the build directory: ./run_standalone.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# binaries sit next to this script in the build dir
BIN_DIR="$SCRIPT_DIR/Release"
if [ ! -d "$BIN_DIR" ]; then
  BIN_DIR="$SCRIPT_DIR"
fi

if [ ! -x "$BIN_DIR/volasim" ] || [ ! -x "$BIN_DIR/lee_control_zmq" ]; then
  echo "[run_standalone] missing binaries — run: cmake -B build && cmake --build build"
  exit 1
fi

cleanup() {
  echo "[run_standalone] shutting down..."
  kill "$SIM_PID" "$CTRL_PID" 2>/dev/null || true
  wait
}
trap cleanup SIGINT SIGTERM

# volasim expects asset paths relative to cwd
cd "$SCRIPT_DIR"

"$BIN_DIR/volasim" &
SIM_PID=$!

"$BIN_DIR/lee_control_zmq" &
CTRL_PID=$!

sleep 3
"$BIN_DIR/send_position" -2.25 2.5 2.0

wait
