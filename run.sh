#!/bin/bash

# Allow X11 connections from Docker
xhost +local:docker

# Run the container with X11 forwarding for Gazebo GUI
docker run -it --rm \
    --name px4_dev \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e QT_X11_NO_MITSHM=1 \
    px4_ros2_gz:latest

# Cleanup X11 permissions after exit
xhost -local:docker
