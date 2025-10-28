FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    git \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    build-essential \
    cmake \
    nano \
    vim \
    sudo \
    locales \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Setup locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add ROS2 repository
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y gz-harmonic && \
    rm -rf /var/lib/apt/lists/*

# Install ROS-Gazebo bridge for Harmonic
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/*

# Install PX4 dependencies
RUN apt-get update && apt-get install -y \
    python3-empy \
    python3-toml \
    python3-numpy \
    python3-yaml \
    python3-dev \
    python3-jinja2 \
    ninja-build \
    exiftool \
    astyle \
    openjdk-11-jdk \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    && rm -rf /var/lib/apt/lists/*

# Install micro-XRCE-DDS agent
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent && \
    cd /tmp/Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig /usr/local/lib/ && \
    rm -rf /tmp/Micro-XRCE-DDS-Agent

# Create workspace
RUN mkdir -p /root/ros2_ws/src

# Clone PX4-Autopilot (latest stable)
WORKDIR /root
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Install PX4 additional dependencies
WORKDIR /root/PX4-Autopilot
RUN bash ./Tools/setup/ubuntu.sh --no-nuttx

# Clone px4_msgs and px4_ros_com
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git

# Clone px4-ros2-interface-lib
RUN git clone --recursive https://github.com/Auterion/px4-ros2-interface-lib.git

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Install workspace dependencies
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"

# Install specific empy version compatible with ROS2 Humble
RUN pip3 install --user empy==3.3.4 pyros-genmsg setuptools

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build"

# Setup environment variables
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export PX4_DIR=/root/PX4-Autopilot" >> /root/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/root/PX4-Autopilot/Tools/simulation/gz/models:\${GZ_SIM_RESOURCE_PATH}" >> /root/.bashrc

# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /root/ros2_ws/install/setup.bash\n\
export PX4_DIR=/root/PX4-Autopilot\n\
export GZ_SIM_RESOURCE_PATH=/root/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}\n\
exec "$@"' > /root/entrypoint.sh && \
    chmod +x /root/entrypoint.sh

WORKDIR /root
ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["/bin/bash"]
