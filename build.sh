#!/bin/bash

# =============================================================================
# PX4 ROS2 Development Container - Build Script
# =============================================================================
# Build the Docker image
# 
# FIRST TIME: Takes 20-30 minutes
# REBUILDS: Only changed layers rebuild (usually seconds/minutes)
#
# Docker uses layer caching:
#   - If Dockerfile unchanged → uses cache → FAST (seconds)
#   - If line changed → rebuilds from that line down → SLOWER
#   - If early line changed → rebuilds everything → SLOWEST
#
# Force full rebuild:
#   docker build --no-cache -t px4_ros2_gz:latest .
#
# Clean cache to free space:
#   docker builder prune
# =============================================================================

echo "Building PX4 ROS2 Gazebo Docker image..."
echo "This uses Docker's layer caching for speed."
echo ""

docker build -t px4_ros2_gz:latest .

echo ""
echo "✅ Build complete!"
echo ""
echo "What just happened:"
echo "  - If build took seconds: Docker used cache (good!)"
echo "  - If build took minutes: Some layers rebuilt (normal)"
echo "  - If build took 20+ min: Full rebuild (Dockerfile changed significantly)"
echo ""
echo "Run the container with: ./run.sh"
