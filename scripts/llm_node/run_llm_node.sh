#!/bin/bash

# Run LLM Node Test with Virtual Environment
echo "🚀 Starting LLM Node..."
echo "================================"

# Set API key
source .env

cd /workspace/M1_WiredUp/ros2_workspace

# Activate virtual environment and ROS2
source ./venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "✅ Environment setup complete!"

# Run the llm_node using virtual environment Python
./venv/bin/python install/llm_node/lib/python3.12/site-packages/llm_node/llm_node.py
