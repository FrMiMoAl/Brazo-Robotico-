#!/bin/bash

echo "Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "Building the workspace..."
colcon build --symlink-install

echo "Sourcing local workspace..."
source install/setup.bash

echo "Environment initialized! You can now run:"
echo "  ros2 launch brazo4 control.launch.py"
echo "  or"
echo "  ros2 launch brazo4 display.launch.py"
