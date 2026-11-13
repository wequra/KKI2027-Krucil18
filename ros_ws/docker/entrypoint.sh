#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

if [ -f /dagozilla_ws/install/setup.bash ]; then
  source /dagozilla_ws/install/setup.bash
fi

exec "$@"
