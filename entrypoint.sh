#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0

exec "$@"