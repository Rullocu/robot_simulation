#!/bin/bash
set -e

# Source ROS 2 setup files
source /opt/ros/humble/setup.bash

# If the workspace has a build, source it too
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# Add automatic sourcing to .bashrc so new terminals get it too
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "# ROS2 environment setup" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "if [ -f \"/ros2_ws/install/setup.bash\" ]; then" >> ~/.bashrc
    echo "    source /ros2_ws/install/setup.bash" >> ~/.bashrc
    echo "fi" >> ~/.bashrc
fi

# Execute the command passed to the script
exec "$@"
