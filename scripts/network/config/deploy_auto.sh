#!/bin/bash
set -e

# Auto-detected Pi configuration
PI_HOSTNAME="pi01.local"
PI_IP="192.168.2.984"
PI_USER="parth"

# Try hostname first, fallback to IP
if ping -c 1 "pi01.local" >/dev/null 2>&1; then
    PI_TARGET="$PI_HOSTNAME"
    echo "✅ Connected via hostname: $PI_HOSTNAME"
else
    PI_TARGET="$PI_IP"
    echo "⚠️  Using IP fallback: $PI_IP"
fi

PI_SSH="$PI_USER@$PI_TARGET"

echo "🚀 Deploying to Pi: $PI_SSH"
echo "📦 Building packages..."

# Build (uncomment if needed)
# cd "/workspace/M1_WiredUp/ros2_workspace"
# colcon build

# Package
cd "/workspace/M1_WiredUp/ros2_workspace/deploy"
rm -f ros2_audio_packages.tar.gz
tar -czf ros2_audio_packages.tar.gz ../install/* ../install/setup.bash ../install/local_setup.bash

# Deploy packages and configs to Pi
echo "📡 Uploading packages and configs..."
scp ros2_audio_packages.tar.gz "$PI_SSH":~/ros2_workspace/deploy/
scp -r ../scripts/network/config "$PI_SSH":~/ros2_workspace/scripts/network/

echo "🔧 Installing on Pi..."
ssh "$PI_SSH" "cd ~/ros2_workspace/deploy && tar -xzf ros2_audio_packages.tar.gz"

echo "✅ Deployment complete!"
echo ""
echo "🔧 Next steps:"
echo "  1. On MacBook: ./scripts/audio_processing_node/run_audio_processing_node.sh"
echo "  2. On Pi: ~/ros2_workspace/scripts/network/config/run_audio_stream_node_pi.sh"
echo ""
echo "🌐 Configuration Summary:"
echo "  - MacBook IP: 172.19.0.2"
echo "  - Pi IP: 192.168.2.984"
echo "  - DDS Server: 172.19.0.2:11811"
echo "  - ROS Domain: 0"
echo "  - Discovery Range: SUBNET"
