
#!/bin/bash
set -e

# ROS2
source /opt/ros/humble/setup.bash

cd /workspace

# if workspace exist, source it
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

exec "$@"