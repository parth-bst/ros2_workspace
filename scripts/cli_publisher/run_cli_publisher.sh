#!/bin/bash

# Run CLI Publisher for LLM Node Communication
echo "🚀 Starting CLI Publisher..."
echo "================================"
echo "This will connect to the LLM node for interactive communication."
echo "Make sure the LLM node is running in another terminal first!"
echo ""

cd /workspace/M1_WiredUp/ros2_workspace

# Activate virtual environment and ROS2
source ./venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "✅ Environment setup complete!"
echo "📡 Available topics:"
ros2 topic list

echo ""
echo "🤖 Starting CLI Publisher..."
echo "Type your messages and press Enter to send them to the LLM node."
echo "Type 'quit', 'exit', or 'q' to stop."
echo "Press Ctrl+C to stop the publisher."
echo "================================"

# Run the CLI publisher
python3 scripts/cli_publisher/cli_publisher.py
