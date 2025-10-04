#!/bin/bash
set -e
PI_IP="parth@pi01.local"

# Build
# rm -rf build/* install/*
# colcon build

# # Packagehe a
# cd deploy
# rm -f ros2_audio_packages.tar.gz
# tar -czf ros2_audio_packages.tar.gz ../install/* ../install/setup.bash ../install/local_setup.bash

# Deploy
scp deploy/ros2_audio_packages.tar.gz "$PI_IP":~/ros2_workspace/deploy/
ssh "$PI_IP" "cd ~/ros2_workspace/deploy && tar -xzf ros2_audio_packages.tar.gz"
