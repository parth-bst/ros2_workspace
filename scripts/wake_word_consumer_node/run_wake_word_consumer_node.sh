#!/bin/bash

echo "run the script from "ros2_workspace" directory only"
# Run Wake Word Consumer Node (for Raspberry Pi)
echo "📱 Starting Wake Word Consumer Node..."
echo "======================================"

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "✅ Environment setup complete!"

# Set logging environment variables for detailed output
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=INFO

# CRITICAL: Set domain 0 for communication
export ROS_DOMAIN_ID=0
# CRITICAL: Set subnet discovery range
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "🔧 Starting Wake Word Consumer Node..."
echo "Listening for results from MacBook:"
echo "  - /wake_word_detected (wake word detections)"
echo "  - /llm_response (LLM responses)"
echo "  - /audio_processing_status (processing status)"
echo "Publishing to:"
echo "  - /pi_local_status (local Pi status updates)"
echo ""

# Run the wake word consumer node
ros2 run wake_word_consumer_node wake_word_consumer_node
