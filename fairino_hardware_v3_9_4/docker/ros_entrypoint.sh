#!/bin/bash
set -e

# Preserve arguments across sourcing (setup.bash may consume $@).
args=( "$@" )
set --

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /opt/grip/install/setup.bash ]; then
  source /opt/grip/install/setup.bash
fi

set -- "${args[@]}"
exec "$@"
