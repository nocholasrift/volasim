#!/usr/bin/env bash
# Launch the ROS2 stack (bridge + controller). The sim runs separately on the
# host. Use --record to capture an MCAP bag of /odometry /command_pos /command
# into <repo>/bags for later inspection in Foxglove Studio / PlotJuggler.
set -e

RECORD=0
TAKEOFF="${AUTO_TAKEOFF:-0}"
COMPOSE_ARGS=()

usage() {
  cat <<EOF
Usage: $0 [options] [-- extra 'docker compose up' args]
  -r, --record    record an MCAP bag to <repo>/bags
  -t, --takeoff   auto-call /takeoff ~2s after startup
  -d, --detach    run the stack in the background
  -h, --help      show this help
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    -r|--record)  RECORD=1; shift ;;
    -t|--takeoff) TAKEOFF=1; shift ;;
    -d|--detach)  COMPOSE_ARGS+=(-d); shift ;;
    -h|--help)    usage; exit 0 ;;
    --)           shift; COMPOSE_ARGS+=("$@"); break ;;
    *)            COMPOSE_ARGS+=("$1"); shift ;;
  esac
done

# Run from the compose file's directory so relative paths resolve.
cd "$(dirname "$0")"

RECORD="$RECORD" AUTO_TAKEOFF="$TAKEOFF" \
  docker compose up --build "${COMPOSE_ARGS[@]}"
