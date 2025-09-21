#!/bin/bash

# Build ROS2 llm_node package with virtual environment
source .env
cd /workspace/M1_WiredUp/ros2_workspace

# Activate virtual environment and build
source ./venv/bin/activate
source /opt/ros/jazzy/setup.bash
colcon build --packages-select llm_node
source install/setup.bash
