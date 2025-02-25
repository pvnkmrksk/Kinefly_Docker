#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/catkin/devel/setup.bash"

# Set RIG environment variable
export RIG=rhag

# Start roscore in the background
roscore &
sleep 2  # Wait for roscore to start

# Launch Kinefly
roslaunch Kinefly main.launch

# Keep container running
exec "$@"
