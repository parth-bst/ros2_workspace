#!/bin/bash

# Build ROS2 llm_node_cpp_basic package
echo "run the script from ros2_workspace directory only"

# Check if .env exists and source it
if [ -f .env ]; then
    source .env
    export OPENAI_API_KEY
else
    echo "⚠️  Warning: .env file not found. Make sure OPENAI_API_KEY is set."
fi

# Build the basic C++ node
echo "🔨 Building llm_node_cpp_basic package..."
source /opt/ros/jazzy/setup.bash
colcon build --packages-select llm_node_cpp_basic

# Source the built packages
source install/setup.bash

echo "✅ Build complete!"
