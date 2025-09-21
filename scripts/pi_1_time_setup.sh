#!/bin/bash

# Pi One-Time Setup Script
echo "🔧 Setting up ROS2 workspace on Pi..."

cd ros2_workspace
python3 -m venv venv
source venv/bin/activate
pip3 install -r src/llm_node/requirements.txt

echo "✅ Pi setup complete!"
