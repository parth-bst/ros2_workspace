#!/bin/bash

echo "🎤 Starting Wake Word Detection Node..."
echo "======================================"

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source /workspace/M1_WiredUp/ros2_workspace/install/setup.bash

echo "✅ Environment setup complete!"

# Set logging environment variables for detailed output
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=INFO
export ROS_DOMAIN_ID=0

echo "🔧 Starting Wake Word Node..."
echo "Listening for keywords: YES, NO, GO, LEFT, RIGHT, UP, DOWN, STOP"
echo "Publishing to:"
echo "  - /llm_input (same as CLI publisher)"
echo "  - /wake_word_detected (raw detection events)"
echo ""

# Run the wake word node
ros2 run wake_word_node wake_word_node
