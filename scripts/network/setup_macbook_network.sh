#!/bin/bash

# Setup ROS2 network configuration for MacBook
echo "💻 Setting up MacBook ROS2 Network Configuration"
echo "==============================================="

echo "🔧 Configuring ROS2 for MacBook (Domain ID: 2)"

# ROS2 Environment Setup for MacBook

export ROS_DOMAIN_ID=2
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=INFO

# Network discovery settings
export ROS_DISCOVERY_SERVER=""
export ROS_DOMAIN_BRIDGE_ENABLED=1

echo "✅ MacBook ROS2 environment configured (Domain ID: 2)"
