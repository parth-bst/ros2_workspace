#!/bin/bash

echo "🚀 Setting up Complete Live Agent System with Deep Agents UI"
echo "============================================================="

# Check if running from correct directory
if [ ! -f "package.xml" ]; then
    echo "❌ Error: Please run this script from the ros2_workspace directory"
    exit 1
fi

echo "📦 Step 1: Building Live Agent ROS2 Package..."
./scripts/live_agent/build_node.sh

if [ $? -ne 0 ]; then
    echo "❌ Failed to build live_agent package"
    exit 1
fi

echo "✅ Live Agent package built successfully!"

echo ""
echo "🌐 Step 2: Setting up Deep Agents UI..."

# Navigate to deep-agents-ui directory
cd src/live_agent/deep-agents-ui

# Check if Node.js is available
if ! command -v node &> /dev/null; then
    echo "❌ Node.js is not installed. Please install Node.js first."
    echo "   Visit: https://nodejs.org/"
    exit 1
fi

# Install UI dependencies
echo "Installing UI dependencies..."
npm install

if [ $? -ne 0 ]; then
    echo "❌ Failed to install UI dependencies"
    exit 1
fi

# Copy configuration
echo "Setting up UI configuration..."
cp ../examples/ui_config.env .env.local

echo "✅ Deep Agents UI setup complete!"

# Return to ros2_workspace
cd ../../..

echo ""
echo "🔧 Step 3: Environment Setup..."

# Check for required environment variables
if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ Error: OPENAI_API_KEY not set in .env file"
    echo "   The agent will not work without this key"
    exit 1
fi

# Set default model if not specified
export OPENAI_MODEL=${OPENAI_MODEL:-"gpt-5-nano"}
echo "✅ Using OpenAI model: $OPENAI_MODEL"

echo ""
echo "🎯 Step 4: System Ready!"
echo "================================"
echo ""
echo "To start the complete system:"
echo ""
echo "1. Start the Live Agent ROS2 node:"
echo "   ./scripts/live_agent/run_live_agent.sh"
echo ""
echo "2. In another terminal, start the Deep Agents UI:"
echo "   cd src/live_agent/deep-agents-ui"
echo "   npm run dev"
echo ""
echo "3. Open your browser to:"
echo "   http://localhost:3000"
echo ""
echo "4. Test the system:"
echo "   ./scripts/live_agent/test_live_agent.sh"
echo ""
echo "🔗 Integration Features:"
echo "   - ROS2 communication with Deep Agents"
echo "   - Web UI for agent interaction"
echo "   - Real-time message history"
echo "   - Tool execution monitoring"
echo "   - Subagent management"
echo ""
echo "✅ Complete system setup finished!"
echo "============================================================="
