#!/bin/bash
# Build the mounted workspace into the colcon volume (not into the host's ros2_ws/build|install).
set -e
source /opt/ros/humble/setup.bash
cd /home/dev/mobile/ros2_ws
colcon --log-base /home/dev/colcon/log build --symlink-install \
  --build-base /home/dev/colcon/build --install-base /home/dev/colcon/install "$@"
echo "built. run: source /home/dev/colcon/install/setup.bash"
