#!/bin/bash

echo "🎧 Starting Wake Word Consumer..."
echo "================================"

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source /workspace/M1_WiredUp/ros2_workspace/install/setup.bash

echo "✅ Environment setup complete!"

# Set logging environment variables for detailed output
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=INFO

# CRITICAL: Set domain 0 for communication
export ROS_DOMAIN_ID=0
# CRITICAL: Set subnet discovery range
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "🔧 Starting Wake Word Consumer..."
echo "Monitoring topics:"
echo "  - /wake_word_detected"
echo "  - /llm_response"
echo ""

# Run the wake word consumer
python3 scripts/wake_word_node/wake_word_consumer.py
