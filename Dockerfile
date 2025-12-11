FROM ubuntu:22.04

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Set locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install basic tools
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    python3-pip \
    sudo \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && apt-get install -y \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y \
    gz-harmonic \
    libgz-sim8-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS-Gazebo bridge packages (Harmonic version)
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/*

# Create user px4user with sudo privileges
RUN useradd -m -s /bin/bash px4user && \
    echo "px4user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER px4user
WORKDIR /home/px4user

# Install PX4 dependencies
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive --branch v1.16.0 && \
    cd PX4-Autopilot && \
    bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools 2>&1 | tee setup.log

# Install additional Python dependencies for PX4
RUN pip3 install --user \
    kconfiglib \
    jinja2 \
    jsonschema \
    pyros-genmsg \
    packaging \
    toml \
    numpy \
    empy==3.3.4

# Build PX4 SITL with Gazebo support (compile only, don't run)
WORKDIR /home/px4user/PX4-Autopilot
RUN bash -c "source /opt/ros/humble/setup.bash && \
    make -j$(nproc) px4_sitl_default" && \
    echo "PX4 build complete"

# Initialize rosdep
RUN sudo rosdep init || true && \
    rosdep update

# Cache buster - change this number to force rebuild from here: v2
# Install Micro-XRCE-DDS Agent (required for PX4-ROS2 communication)
WORKDIR /home/px4user
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    sudo make install && \
    sudo ldconfig /usr/local/lib/

# Create ROS2 workspace
WORKDIR /home/px4user
RUN mkdir -p ros2_ws/src

# Clone px4_msgs matching PX4 v1.16
WORKDIR /home/px4user/ros2_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git --branch release/1.16

# Clone px4_ros_com
RUN git clone https://github.com/PX4/px4_ros_com.git --branch release/1.16

# Clone px4-ros2-interface-lib matching PX4 v1.16
RUN git clone https://github.com/Auterion/px4-ros2-interface-lib.git --branch release/1.16

# Build ROS2 workspace
WORKDIR /home/px4user/ros2_ws
RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build"

# Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> /home/px4user/.bashrc && \
    echo "source /home/px4user/ros2_ws/install/setup.bash" >> /home/px4user/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/home/px4user/PX4-Autopilot/Tools/simulation/gz/models" >> /home/px4user/.bashrc

# Set working directory to ros2_ws for development
WORKDIR /home/px4user/ros2_ws

CMD ["/bin/bash"]
