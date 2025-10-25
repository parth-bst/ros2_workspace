#!/bin/bash

# Build ROS2 live_agent package with virtual environment
echo "Building live_agent package..."
echo "run the script from 'ros2_workspace' directory only"

# Source environment
source .env
export OPENAI_API_KEY

# Activate virtual environment and build
source ./venv/bin/activate

# Install deepagents and dependencies
echo "Installing deepagents and dependencies..."
pip install deepagents
pip install langchain langchain-core langchain-community langgraph
pip install langchain-openai
pip install tavily-python
pip install -r src/live_agent/requirements.txt

# Build the package
echo "Building live_agent package..."
source /opt/ros/jazzy/setup.bash
colcon build --packages-select live_agent
source install/setup.bash

echo "✅ live_agent package built successfully!"
