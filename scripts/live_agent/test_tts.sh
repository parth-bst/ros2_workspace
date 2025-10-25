#!/bin/bash

echo "run the script from 'ros2_workspace' directory only"
# Test TTS functionality specifically
echo "🎤 Testing TTS functionality..."
echo "================================"

# Set API keys and model
source .env
export OPENAI_API_KEY
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

echo "🎤 Testing TTS tool specifically..."

# Test 1: Simple TTS request
echo "Test 1: Simple TTS request..."
ros2 topic pub --once /live_agent/user_text std_msgs/String "data: 'Say hello world'"

sleep 3

# Test 2: TTS with longer text
echo "Test 2: TTS with longer text..."
ros2 topic pub --once /live_agent/user_text std_msgs/String "data: 'Please say: This is a test of the text to speech system'"

sleep 3

# Test 3: TTS with question
echo "Test 3: TTS with question..."
ros2 topic pub --once /live_agent/user_text std_msgs/String "data: 'Can you say your name and what you can do?'"

echo "✅ TTS tests completed! Check the /tts/speak topic for output."
echo "================================"

sleep 3

# Test 3: TTS with question
echo "Test 4: light LED..."
ros2 topic pub --once /live_agent/user_text std_msgs/String "data: 'Please blink the LED for 5 seconds'"

echo "✅ TTS tests completed! Check the /tts/speak topic for output."
echo "================================"