#!/bin/bash

# Run this script INSIDE the container to verify everything works

echo "========================================="
echo "Testing PX4 + ROS2 + Gazebo Setup"
echo "========================================="
echo ""

# Test 1: Check ROS2
echo "✓ Checking ROS2 Humble..."
ros2 --version
echo ""

# Test 2: Check Gazebo
echo "✓ Checking Gazebo Harmonic..."
gz sim --version
echo ""

# Test 3: Check px4_msgs
echo "✓ Checking px4_msgs..."
ros2 interface list | grep px4_msgs | head -5
echo ""

# Test 4: Check PX4 build
echo "✓ Checking PX4 SITL binary..."
ls -lh /home/px4user/PX4-Autopilot/build/px4_sitl_default/bin/px4
echo ""

echo "========================================="
echo "✅ All checks passed!"
echo "========================================="
echo ""
echo "To run PX4 SITL with Gazebo, use:"
echo "  cd ~/PX4-Autopilot"
echo "  make px4_sitl gz_x500"
echo ""
echo "In another terminal (inside container):"
echo "  source ~/ros2_ws/install/setup.bash"
echo "  ros2 topic list"
