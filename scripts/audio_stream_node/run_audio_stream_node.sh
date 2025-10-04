#!/bin/bash

echo "run the script from "ros2_workspace" directory only"

# Auto-detect network and generate DDS configs
echo "🔍 Auto-detecting network configuration..."
./scripts/network/auto_detect_network.sh

# Source the generated config
export FASTRTPS_DEFAULT_PROFILES_FILE=../scripts/network/config/dds/discovery_client_pi.xml

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

echo "🔧 Starting Audio Stream Node (LOW-LATENCY MODE)..."
echo "Streaming audio from re-speaker to:"
echo "  - /audio_stream (raw audio data)"
echo "  - /audio_stream_status (status updates)"
echo "Listening for responses:"
echo "  - /wake_word_detected (from MacBook)"
echo "  - /llm_response (from MacBook)"
echo "⚡ Low-latency optimizations:"
echo "  - Chunk size: 256 samples (16ms)"
echo "  - Streaming interval: 1ms"
echo "  - Optimized for minimal capture-to-transmit delay"
echo ""
echo "🌐 DDS Discovery Client connecting to: $(grep -oP '<address>\K[^<]+' scripts/network/config/dds/discovery_client_pi.xml):11811"
echo "🔗 ROS Domain ID: $ROS_DOMAIN_ID"
echo "🌍 Discovery Range: $ROS_AUTOMATIC_DISCOVERY_RANGE"
echo ""

# Run the audio stream node using direct Python module path
./venv/bin/python install/audio_stream_node/lib/python3.12/site-packages/audio_stream_node/audio_stream_node.py
