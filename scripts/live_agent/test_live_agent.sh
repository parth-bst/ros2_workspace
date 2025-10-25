#!/bin/bash

echo "run the script from 'ros2_workspace' directory only"
# Test Live Agent Node
echo "🧪 Testing Live Agent Node..."
echo "================================"

# Set API keys and model
source .env
export OPENAI_API_KEY
export TAVILY_API_KEY
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
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=INFO

# CRITICAL: Set domain 0 for communication
export ROS_DOMAIN_ID=0
# CRITICAL: Set subnet discovery range
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "🧪 Running Live Agent tests..."

# Test 1: Basic text input
echo "Test 1: Sending basic text input..."
ros2 topic pub --once /live_agent/user_text std_msgs/String "data: 'Hello, can you introduce yourself?'"

sleep 2

# Test 2: System status request
echo "Test 2: Requesting system status..."
ros2 topic pub --once /live_agent/user_text std_msgs/String "data: 'What is the current system status?'"

sleep 2

# Test 3: TTS request
echo "Test 3: Requesting TTS..."
ros2 topic pub --once /live_agent/user_text std_msgs/String "data: 'Say hello to me'"

sleep 2

# Test 4: Safety check request
echo "Test 4: Requesting safety check..."
ros2 topic pub --once /live_agent/user_text std_msgs/String "data: 'Can you perform a safety check?'"

sleep 2

# Test 5: General conversation
echo "Test 5: General conversation..."
ros2 topic pub --once /live_agent/user_text std_msgs/String "data: 'Tell me about your capabilities'"

echo "✅ Tests completed! Check the agent responses above."
echo "================================"
