#!/bin/bash

# Setup ROS2 network configuration for Raspberry Pi
echo "🍓 Setting up Raspberry Pi ROS2 Network Configuration"
echo "===================================================="

echo "🔧 Configuring ROS2 for Raspberry Pi (Domain ID: 1)"

# ROS2 Environment Setup for Raspberry Pi

export ROS_DOMAIN_ID=1
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=INFO

# Network discovery settings
export ROS_DISCOVERY_SERVER=""
export ROS_DOMAIN_BRIDGE_ENABLED=1

echo "✅ Raspberry Pi ROS2 environment configured (Domain ID: 1)"
