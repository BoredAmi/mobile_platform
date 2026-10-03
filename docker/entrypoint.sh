#!/bin/bash
# Source ROS and, if it has been built, the workspace (built into /home/dev/colcon by build_ws).
source /opt/ros/humble/setup.bash
if [ -f /home/dev/colcon/install/setup.bash ]; then
  source /home/dev/colcon/install/setup.bash
fi
grep -q "entrypoint.sh" ~/.bashrc 2>/dev/null || cat >> ~/.bashrc <<'RC'
source /opt/ros/humble/setup.bash
[ -f /home/dev/colcon/install/setup.bash ] && source /home/dev/colcon/install/setup.bash  # entrypoint.sh
RC
exec "$@"
