#!/bin/bash

echo "run the script from "ros2_workspace" directory only"
# Run Audio Stream Node (for Raspberry Pi)
echo "🎤 Starting Audio Stream Node..."
echo "================================"

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "✅ Environment setup complete!"

# Set logging environment variables for detailed output
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=INFO
export ROS_DOMAIN_ID=0

echo "🔧 Starting Audio Stream Node..."
echo "Streaming audio from re-speaker to:"
echo "  - /audio_stream (raw audio data)"
echo "  - /audio_stream_status (status updates)"
echo "Listening for responses:"
echo "  - /wake_word_detected (from MacBook)"
echo "  - /llm_response (from MacBook)"
echo ""

# Run the audio stream node
ros2 run audio_stream_node audio_stream_node
