#!/bin/bash

# Build ROS2 wake_word_consumer_node package
echo "run the script from "ros2_workspace" directory only"

# Source ROS2 environment and build
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wake_word_consumer_node
source install/setup.bash
