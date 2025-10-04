#!/bin/bash

# Auto-detect network configuration for DDS discovery
# This script detects the current network and generates appropriate DDS configs
# Updates both MacBook and Pi configurations with domain 0 and subnet discovery

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONFIG_DIR="$SCRIPT_DIR/config/dds"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Detect current network interface and IP
detect_network() {
    log_info "Detecting network configuration..."
    
    # Get primary network interface (WiFi or Ethernet)
    if command -v ip >/dev/null 2>&1; then
        PRIMARY_INTERFACE=$(ip route | grep default | head -1 | awk '{print $5}')
        LOCAL_IP=$(ip route get 8.8.8.8 | grep -oP 'src \K\S+')
    elif command -v route >/dev/null 2>&1; then
        PRIMARY_INTERFACE=$(route -n | grep '^0.0.0.0' | head -1 | awk '{print $8}')
        LOCAL_IP=$(ifconfig $PRIMARY_INTERFACE | grep 'inet ' | awk '{print $2}' | cut -d: -f2)
    else
        log_error "Cannot detect network interface"
        exit 1
    fi
    
    # Extract network prefix (e.g., 192.168.2 from 192.168.2.9)
    NETWORK_PREFIX=$(echo $LOCAL_IP | cut -d. -f1-3)
    
    log_success "Network detected: $LOCAL_IP on interface $PRIMARY_INTERFACE"
    log_info "Network prefix: $NETWORK_PREFIX"
    
    # Export variables for use by other scripts
    export LOCAL_IP
    export NETWORK_PREFIX
    export PRIMARY_INTERFACE
}

# Find Pi on the network
find_pi() {
    log_info "Searching for Pi on network $NETWORK_PREFIX.x..."
    
    # Try common Pi hostnames first
    for hostname in pi01.local raspberrypi.local raspberrypi pi01; do
        if ping -c 1 -W 2 "$hostname" >/dev/null 2>&1; then
            PI_HOSTNAME="$hostname"
            PI_IP=$(ping -c 1 "$hostname" | head -1 | grep -oP '\(\K[^)]+' | tr -d '\n')
            log_success "Found Pi via hostname: $PI_HOSTNAME ($PI_IP)"
            export PI_HOSTNAME
            export PI_IP
            return 0
        fi
    done
    
    # Scan network for Pi (look for SSH on port 22)
    log_info "Scanning network for Pi..."
    for i in {1..254}; do
        IP="$NETWORK_PREFIX.$i"
        if ping -c 1 -W 1 "$IP" >/dev/null 2>&1; then
            # Check if it's a Pi by trying SSH with common usernames
            for user in parth pi raspberry; do
                if ssh -o ConnectTimeout=3 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
                   "$user@$IP" "hostname" >/dev/null 2>&1; then
                    PI_IP="$IP"
                    PI_USER="$user"
                    log_success "Found Pi at $PI_IP (user: $PI_USER)"
                    export PI_IP
                    export PI_USER
                    return 0
                fi
            done
        fi
    done
    
    log_error "Pi not found on network $NETWORK_PREFIX.x"
    return 1
}

# Generate DDS configuration files with domain 0 and subnet discovery
generate_dds_configs() {
    log_info "Generating DDS configuration files with domain 0 and subnet discovery..."
    
    # Create config directory if it doesn't exist
    mkdir -p "$CONFIG_DIR"
    
    # Generate MacBook Discovery Server config with Large Data Mode
    cat > "$CONFIG_DIR/discovery_server_macbook.xml" << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="discovery_server_macbook">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SERVER</discoveryProtocol>
                </discovery_config>
                <metatrafficUnicastLocatorList>
                    <locator>
                        <udpv4>
                            <address>$LOCAL_IP</address>
                            <port>11811</port>
                        </udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
                <useBuiltinTransports>false</useBuiltinTransports>
            </builtin>
            <userTransports>
                <transport_id>udp_transport</transport_id>
                <transport_id>tcp_transport</transport_id>
            </userTransports>
        </rtps>
        <transports>
            <transport_descriptors>
                <transport_descriptor>
                    <transport_id>udp_transport</transport_id>
                    <type>UDPv4</type>
                </transport_descriptor>
                <transport_descriptor>
                    <transport_id>tcp_transport</transport_id>
                    <type>TCPv4</type>
                    <tcp_nodelay>true</tcp_nodelay>
                    <maxMessageSize>65500</maxMessageSize>
                    <maxInitialPeersRange>16</maxInitialPeersRange>
                </transport_descriptor>
            </transport_descriptors>
        </transports>
    </participant>
</profiles>
EOF

    # Generate Pi Discovery Client config
    cat > "$CONFIG_DIR/discovery_client_pi.xml" << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="discovery_client_pi">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>CLIENT</discoveryProtocol>
                    <discoveryServersList>
                        <locator>
                            <udpv4>
                                <address>$LOCAL_IP</address>
                                <port>11811</port>
                            </udpv4>
                        </locator>
                    </discoveryServersList>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
EOF

    log_success "DDS configuration files generated:"
    log_info "  - $CONFIG_DIR/discovery_server_macbook.xml"
    log_info "  - $CONFIG_DIR/discovery_client_pi.xml"
}

# Update MacBook run scripts with domain 0 and DDS config
update_macbook_scripts() {
    log_info "Updating MacBook run scripts with domain 0 and DDS config..."
    
    # Update audio processing node script
    cat > "$WORKSPACE_ROOT/scripts/audio_processing_node/run_audio_processing_node.sh" << 'EOF'
#!/bin/bash

echo "run the script from "ros2_workspace" directory only"

# Auto-detect network and generate DDS configs
echo "🔍 Auto-detecting network configuration..."
./scripts/network/auto_detect_network.sh

# Source the generated config
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/M1_WiredUp/ros2_workspace/scripts/network/config/dds/discovery_server_macbook.xml

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

# Run the audio processing node
./venv/bin/python install/audio_processing_node/lib/python3.12/site-packages/audio_processing_node/audio_processing_node.py
EOF

    chmod +x "$WORKSPACE_ROOT/scripts/audio_processing_node/run_audio_processing_node.sh"
    
    log_success "Updated MacBook audio processing node script"
}

# Update Pi run scripts with domain 0 and DDS config
update_pi_scripts() {
    log_info "Updating Pi run scripts with domain 0 and DDS config..."
    
    # Create Pi audio stream node script
    cat > "$SCRIPT_DIR/config/run_audio_stream_node_pi.sh" << 'EOF'
#!/bin/bash

echo "run the script from "ros2_workspace" directory only"

# Source ROS2 environment
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
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/parth/ros2_workspace/scripts/network/config/dds/discovery_client_pi.xml

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

# Run the audio stream node
python3 install/audio_stream_node/lib/python3.12/site-packages/audio_stream_node/audio_stream_node.py
EOF

    chmod +x "$CONFIG_DIR/run_audio_stream_node_pi.sh"
    
    log_success "Created Pi audio stream node script"
}

# Create deploy script that updates both MacBook and Pi
create_deploy_script() {
    if [ -n "$PI_IP" ]; then
        log_info "Creating deploy script that updates both MacBook and Pi..."
        
        # Create a comprehensive deploy script in network/config directory
        cat > "$SCRIPT_DIR/config/deploy_auto.sh" << EOF
#!/bin/bash
set -e

# Auto-detected Pi configuration
PI_HOSTNAME="${PI_HOSTNAME:-pi01.local}"
PI_IP="$PI_IP"
PI_USER="${PI_USER:-parth}"

# Try hostname first, fallback to IP
if ping -c 1 "$PI_HOSTNAME" >/dev/null 2>&1; then
    PI_TARGET="\$PI_HOSTNAME"
    echo "✅ Connected via hostname: \$PI_HOSTNAME"
else
    PI_TARGET="\$PI_IP"
    echo "⚠️  Using IP fallback: \$PI_IP"
fi

PI_SSH="\$PI_USER@\$PI_TARGET"

echo "🚀 Deploying to Pi: \$PI_SSH"
echo "📦 Building packages..."

# Build (uncomment if needed)
# cd "$WORKSPACE_ROOT"
# colcon build

# Package
cd "$WORKSPACE_ROOT/deploy"
rm -f ros2_audio_packages.tar.gz
tar -czf ros2_audio_packages.tar.gz ../install/* ../install/setup.bash ../install/local_setup.bash

# Deploy packages and configs to Pi
echo "📡 Uploading packages and configs..."
scp ros2_audio_packages.tar.gz "\$PI_SSH":~/ros2_workspace/deploy/
scp -r ../scripts/network/config "\$PI_SSH":~/ros2_workspace/scripts/network/

echo "🔧 Installing on Pi..."
ssh "\$PI_SSH" "cd ~/ros2_workspace/deploy && tar -xzf ros2_audio_packages.tar.gz"

echo "✅ Deployment complete!"
echo ""
echo "🔧 Next steps:"
echo "  1. On MacBook: ./scripts/audio_processing_node/run_audio_processing_node.sh"
echo "  2. On Pi: ~/ros2_workspace/scripts/network/config/run_audio_stream_node_pi.sh"
echo ""
echo "🌐 Configuration Summary:"
echo "  - MacBook IP: $LOCAL_IP"
echo "  - Pi IP: $PI_IP"
echo "  - DDS Server: $LOCAL_IP:11811"
echo "  - ROS Domain: 0"
echo "  - Discovery Range: SUBNET"
EOF
        
        chmod +x "$SCRIPT_DIR/config/deploy_auto.sh"
        log_success "Auto-deploy script created: scripts/network/config/deploy_auto.sh"
    fi
}

# Main execution
main() {
    echo "🔍 Auto Network Detection & DDS Configuration"
    echo "=============================================="
    
    detect_network
    
    if find_pi; then
        generate_dds_configs
        update_macbook_scripts
        update_pi_scripts
        create_deploy_script
        
        echo ""
        log_success "✅ Auto-configuration complete!"
        echo ""
        echo "📋 Summary:"
        echo "  - Local IP: $LOCAL_IP"
        echo "  - Pi IP: $PI_IP"
        echo "  - DDS Server: $LOCAL_IP:11811"
        echo "  - ROS Domain: 0"
        echo "  - Discovery Range: SUBNET"
        echo "  - Config files: scripts/network/config/dds/"
        echo ""
        echo "🚀 Next steps:"
        echo "  1. Deploy to Pi: ./scripts/network/config/deploy_auto.sh"
        echo "  2. Start MacBook node: ./scripts/audio_processing_node/run_audio_processing_node.sh"
        echo "  3. Start Pi node: ~/ros2_workspace/scripts/network/config/run_audio_stream_node_pi.sh"
        
    else
        log_error "❌ Cannot proceed without Pi connection"
        exit 1
    fi
}

# Run main function
main "$@"
