#!/bin/bash

echo "run the script from 'ros2_workspace' directory only"
# Run Live Agent Node with Virtual Environment
echo "🚀 Starting Live Agent Node..."
echo "================================"

# Set API keys and model
source .env
export OPENAI_API_KEY
export TAVILY_API_KEY

# Set OpenAI model (default to gpt-5-nano if not specified)
export OPENAI_MODEL=${OPENAI_MODEL:-"gpt-5-nano"}

# Activate virtual environment and ROS2
source ./venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "✅ Environment setup complete!"

# Set logging environment
export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# CRITICAL: Set domain 0 for communication
export ROS_DOMAIN_ID=0
# CRITICAL: Set subnet discovery range
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "🤖 Starting Live Agent with Deep Agents integration..."
echo "📡 Listening on topics:"
echo "   - Input: /live_agent/user_text"
echo "   - Output: /live_agent/response"
echo "   - Status: /live_agent/status"
echo ""
echo "🔧 Available tools:"
echo "   - Text-to-speech communication"
echo "   - System status monitoring"
echo "   - Safety checking"
echo "   - Intelligent conversation"
echo "   - Long-term memory"
echo ""
echo "Press Ctrl+C to stop"
echo "================================"

# Run the live_agent node
./venv/bin/python install/live_agent/lib/python3.12/site-packages/live_agent/live_agent.py
