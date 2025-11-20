#!/bin/bash

# Build with progress output
echo "Building with debug output..."
echo "You should see build progress messages"
echo ""

docker build --progress=plain -f Dockerfile.debug -t px4_ros2_gz:latest .

echo ""
echo "✅ Build complete!"
echo "Run the container with: ./run.sh"
