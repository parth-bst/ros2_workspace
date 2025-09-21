#!/bin/bash

echo "run the script from "ros2_workspace" directory only"
# Run LLM Node Test with Virtual Environment
echo "🚀 Starting LLM Node..."
echo "================================"

# Set API key
source .env
export OPENAI_API_KEY

# Activate virtual environment and ROS2
source ./venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "✅ Environment setup complete!"

# Run the llm_node using virtual environment Python with detailed logging
export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
export ROS_DOMAIN_ID=0

./venv/bin/python install/llm_node/lib/python3.12/site-packages/llm_node/llm_node.py
