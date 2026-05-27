#script bash exectued when the container starts

#!/bin/bash
set -e # stop container if error

# Source ROS2
source /opt/ros/humble/setup.bash

# Source ws if built
if [ -f /root/tbot3_nav_monitor/install/setup.bash ]; then
    source /root/tbot3_nav_monitor/install/setup.bash
fi

# Execute the container comand
exec "$@"