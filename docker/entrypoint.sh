#!/usr/bin/env bash
# Source ROS and the workspace overlay, then exec the command (ADR-5).
# setup.bash is sourced, never exec'd — that is the whole point of this file.
set -e
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
