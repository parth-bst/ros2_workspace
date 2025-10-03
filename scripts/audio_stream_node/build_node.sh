#!/bin/bash

# Build ROS2 audio_stream_node package
echo "run the script from "ros2_workspace" directory only"

# Source ROS2 environment and build
source /opt/ros/jazzy/setup.bash
colcon build --packages-select audio_stream_node
source install/setup.bash
