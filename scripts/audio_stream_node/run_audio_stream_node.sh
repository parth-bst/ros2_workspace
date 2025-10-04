#!/bin/bash

echo "run the script from "ros2_workspace" directory only"
# Run Audio Stream Node (for Raspberry Pi)
echo "🎤 Starting Audio Stream Node..."
echo "================================"

source .env
export OPENAI_API_KEY
source ./venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "✅ Environment setup complete!"

# Set logging environment variables for detailed output
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=INFO

# CRITICAL: Set domain 0 for communication with MacBook
export ROS_DOMAIN_ID=0
# CRITICAL: Set subnet discovery range
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Source the DDS client config
export FASTRTPS_DEFAULT_PROFILES_FILE=../scripts/network/config/dds/discovery_client_pi.xml
echo "🌐 Using DDS client config: ../scripts/network/config/dds/discovery_client_pi.xml"

echo "🔧 Starting Audio Stream Node (LOW-LATENCY MODE)..."
echo "Streaming audio to MacBook (Domain 0):"
echo "  - /audio_stream (raw audio data to MacBook)"
echo "Listening for responses:"
echo "  - /wake_word_detected (from MacBook)"
echo "  - /llm_response (from MacBook)"
echo "⚡ Low-latency audio streaming:"
echo "  - Chunk size: 256 samples (16ms)"
echo "  - Streaming interval: 1ms"
echo ""
echo "🌐 DDS Discovery Client connecting to: $(grep -oP '<address>\K[^<]+' scripts/network/config/dds/discovery_client_pi.xml):11811"
echo "🔗 ROS Domain ID: $ROS_DOMAIN_ID"
echo "🌍 Discovery Range: $ROS_AUTOMATIC_DISCOVERY_RANGE"
echo ""

# Run the audio stream node using direct Python module path
./venv/bin/python install/audio_stream_node/lib/python3.12/site-packages/audio_stream_node/audio_stream_node.py
