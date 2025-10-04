#!/bin/bash

# Build ROS2 llm_node package with virtual environment
echo "run the script from "ros2_workspace" directory only"
source .env
export OPENAI_API_KEY

# Activate virtual environment and build
source ./venv/bin/activate
pip3 install -y -r ../src/audio_stream_node/requirements.txt
source /opt/ros/jazzy/setup.bash
colcon build --packages-select audio_stream_node
source install/setup.bash
