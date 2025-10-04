#!/bin/bash

# Audio Listener Runner Script
# Runs the audio consumer that plays audio from the /audio_stream topic

set -e

echo "🎧 Starting Audio Stream Listener..."
echo "=================================="

# Check if we're in the right directory
if [ ! -f "scripts/audio_consumer/audio_listener.py" ]; then
    echo "❌ Error: audio_listener.py not found!"
    echo "Please run this script from the ros2_workspace root directory"
    exit 1
fi

# Source ROS2 environment
echo "📡 Sourcing ROS2 environment..."
source /opt/ros/jazzy/setup.bash

# Build the workspace if needed
echo "🔨 Building workspace..."
colcon build --packages-select audio_msgs

# Source the workspace
echo "🔧 Sourcing workspace..."
source install/setup.bash

# Check if audio_stream topic exists
echo "🔍 Checking for audio stream..."
if ! ros2 topic list | grep -q "/audio_stream"; then
    echo "❌ Error: /audio_stream topic not found!"
    echo "Make sure the audio_stream_node is running first."
    echo "Run: ros2 run audio_stream_node audio_stream_node"
    exit 1
fi

# Check if topic has a publisher
PUBLISHER_COUNT=$(ros2 topic info /audio_stream | grep "Publisher count" | awk '{print $3}')
if [ "$PUBLISHER_COUNT" -eq 0 ]; then
    echo "❌ Error: No publisher found for /audio_stream topic!"
    echo "Make sure the audio_stream_node is running first."
    exit 1
fi

echo "✅ Audio stream found with $PUBLISHER_COUNT publisher(s)"
echo ""

# Run the audio listener
echo "🎵 Starting audio listener..."
echo "You should hear audio from your re-speaker microphone."
echo "Press Ctrl+C to stop."
echo ""

python3 scripts/audio_consumer/audio_listener.py
