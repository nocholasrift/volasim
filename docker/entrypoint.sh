#!/usr/bin/env bash
set -e

# ROS_DISTRO comes from the base image, so this is shared across ROS versions.
source "/opt/ros/${ROS_DISTRO}/setup.bash"

exec "$@"
