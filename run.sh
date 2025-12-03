#!/bin/bash

# Allow X11 access
xhost +local:docker

docker run -it --rm \
    --name px4_dev \
    --privileged \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e __VK_LAYER_NV_optimus=NVIDIA_only \
    -e QT_X11_NO_MITSHM=1 \
    --device=/dev/dri:/dev/dri \
    --group-add video \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/px4user/.Xauthority:rw \
    -v $HOME/px4-sim-models:/home/px4user/PX4-Autopilot/Tools/simulation/gz/models:rw \
    --network host \
    px4_ros2_gz:latest
