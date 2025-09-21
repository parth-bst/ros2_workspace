#!/bin/bash

echo "run the script from "ros2_workspace" directory only"
# Run LLM Node C++ Basic Test
echo "🚀 Starting LLM Node C++ Basic..."
echo "================================"

# Set API key
if [ -f .env ]; then
    source .env
    export OPENAI_API_KEY
else
    echo "⚠️  Warning: .env file not found. Make sure OPENAI_API_KEY is set."
fi

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "✅ Environment setup complete!"

# Set logging environment variables for detailed output
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
export ROS_DOMAIN_ID=0

echo "🔧 Starting C++ LLM Node (Basic)..."
echo "Listening on topics:"
echo "  - /llm_input (std_msgs/String)"
echo "Publishing to:"
echo "  - /llm_response (std_msgs/String)"
echo ""

# Run the C++ node
ros2 run llm_node_cpp_basic llm_node_cpp_basic
