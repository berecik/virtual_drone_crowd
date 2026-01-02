#!/bin/bash
set -e

# Source ROS 2 Humble
source "/opt/ros/humble/setup.bash"

# Source the workspace if it exists
if [ -f "/root/sar_swarm_ws/install/setup.bash" ]; then
    source "/root/sar_swarm_ws/install/setup.bash"
fi

# Set ROS_DOMAIN_ID if provided, else default to 0
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

exec "$@"
