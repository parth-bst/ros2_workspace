#!/bin/bash

echo "run the script from "ros2_workspace" directory only"

# Auto-detect network and generate DDS configs
echo "🔍 Auto-detecting network configuration..."
./scripts/network/auto_detect_network.sh

# Start Discovery Server
echo "🚀 Starting Fast DDS Discovery Server..."
HOST_IP=$(grep -oP '<address>\K[^<]+' scripts/network/config/dds/discovery_server_macbook.xml)
echo "🌐 Discovery Server IP: $HOST_IP:11811"
fastdds discovery -t $HOST_IP -q 11811 &
DISCOVERY_SERVER_PID=$!
echo "✅ Discovery Server started (PID: $DISCOVERY_SERVER_PID)"

# Source the generated config
export FASTRTPS_DEFAULT_PROFILES_FILE=../scripts/network/config/dds/discovery_server_macbook.xml

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

# CRITICAL: Set domain 0 for communication with Pi
export ROS_DOMAIN_ID=0
# CRITICAL: Set subnet discovery range
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

echo "🔧 Starting Audio Processing Node (LOW-LATENCY MODE)..."
echo "Processing audio streams from Pi (Domain 0):"
echo "  - /audio_stream (raw audio data from Pi)"
echo "Publishing results:"
echo "  - /wake_word_detected (to Pi)"
echo "  - /llm_input (to LLM node)"
echo "  - /audio_processing_status (status updates)"
echo "Listening for:"
echo "  - /llm_response (from LLM node)"
echo "⚡ Low-latency audio playback:"
echo "  - Playing audio through MacBook speakers (optimized for minimal delay)"
echo "  - Buffer size: 100ms chunks"
echo "  - Playback chunk: 16ms"
echo "  - Processing interval: 100ms"
echo ""
echo "🌐 DDS Discovery Server: $(grep -oP '<address>\K[^<]+' scripts/network/config/dds/discovery_server_macbook.xml):11811"
echo "🔗 ROS Domain ID: $ROS_DOMAIN_ID"
echo "🌍 Discovery Range: $ROS_AUTOMATIC_DISCOVERY_RANGE"
echo ""

# Cleanup function
cleanup() {
    echo "🧹 Cleaning up Discovery Server..."
    if [ ! -z "$DISCOVERY_SERVER_PID" ]; then
        kill $DISCOVERY_SERVER_PID 2>/dev/null
        echo "✅ Discovery Server stopped"
    fi
}

# Set trap for cleanup on exit
trap cleanup EXIT

# Run the audio processing node
./venv/bin/python install/audio_processing_node/lib/python3.12/site-packages/audio_processing_node/audio_processing_node.py
