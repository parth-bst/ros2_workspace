#!/bin/bash

echo "run the script from "ros2_workspace" directory only"
# Run Audio Processing Node (for MacBook)
echo "🧠 Starting Audio Processing Node..."
echo "===================================="

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "✅ Environment setup complete!"

# Set logging environment variables for detailed output
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=INFO
export ROS_DOMAIN_ID=0

echo "🔧 Starting Audio Processing Node..."
echo "Processing audio streams from Pi:"
echo "  - /audio_stream (raw audio data from Pi)"
echo "Publishing results:"
echo "  - /wake_word_detected (to Pi)"
echo "  - /llm_input (to LLM node)"
echo "  - /audio_processing_status (status updates)"
echo "Listening for:"
echo "  - /llm_response (from LLM node)"
echo ""

# Run the audio processing node
ros2 run audio_processing_node audio_processing_node
