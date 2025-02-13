#!/bin/bash

set -e
cd /arena_camera_ros2/ros2_ws 
#rosdep fix-permissions
#rosdep update
rosdep install --from-paths src --ignore-src -r -y;
ls install
source /opt/ros/eloquent/install/setup.bash && colcon build --symlink-install
source install/local_setup.bash

exec "$@"
