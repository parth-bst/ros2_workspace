#!/bin/bash

# Build ROS2 wake_word_node package
echo "run the script from "ros2_workspace" directory only"

# Check if .env file exists and source it
if [ -f .env ]; then
    source .env
    echo "✅ Loaded environment variables from .env"
else
    echo "⚠️  Warning: .env file not found. Make sure OPENAI_API_KEY is set if needed."
fi

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Build the wake_word_node package
echo "🔧 Building wake_word_node package..."
colcon build --packages-select wake_word_node

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "✅ wake_word_node package built successfully!"
    source install/setup.bash
    echo "✅ ROS2 workspace sourced"
else
    echo "❌ Build failed!"
    exit 1
fi

echo ""
echo "🎤 Wake word node build complete!"
echo "Run with: ros2 run wake_word_node wake_word_node"
