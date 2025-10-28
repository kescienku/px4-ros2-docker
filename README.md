# PX4 + ROS2 Humble + Gazebo Harmonic Docker Environment

Complete guide for running PX4 simulation with ROS2 Humble and Gazebo Harmonic in Docker.

## Table of Contents
- [System Requirements](#system-requirements)
- [Initial Setup](#initial-setup)
- [Docker Commands Reference](#docker-commands-reference)
- [Running the PX4 Example](#running-the-px4-example)
- [Common Workflows](#common-workflows)
- [Troubleshooting](#troubleshooting)
- [Development Tips](#development-tips)

---

## System Requirements

- **OS**: Linux (Ubuntu 20.04/22.04 recommended)
- **Docker**: Version 20.10 or higher
- **RAM**: Minimum 8GB (16GB recommended)
- **Disk Space**: At least 25GB free
- **Display**: X11 server for GUI applications

---

## Initial Setup

### 1. Install Docker (if not already installed)

```bash
# Update package list
sudo apt-get update

# Install Docker
sudo apt-get install docker.io

# Add your user to docker group (to run without sudo)
sudo usermod -aG docker $USER

# Log out and log back in for group changes to take effect
# Or run: newgrp docker
```

### 2. Verify Docker Installation

```bash
docker --version
docker compose version
```

### 3. Project Structure

Create your project directory:

```bash
mkdir -p ~/px4-ros2-docker
cd ~/px4-ros2-docker
```

Your directory should contain:
```
px4-ros2-docker/
├── Dockerfile
├── compose.yaml (or docker-compose.yml)
├── workspace/          # Your custom code goes here
└── README.md           # This file
```

### 4. Enable X11 for GUI Applications

```bash
# Allow Docker to access your display
xhost +local:docker
```

**Note**: Add this to your `~/.bashrc` to make it permanent:
```bash
echo "xhost +local:docker > /dev/null 2>&1" >> ~/.bashrc
```

---

## Docker Commands Reference

### Building the Container

#### First Time Build
```bash
# Build the container (takes 25-35 minutes)
docker compose build
```

#### Rebuild After Changes
```bash
# Rebuild without using cache (clean build)
docker compose build --no-cache

# Rebuild a specific service
docker compose build px4-ros2
```

### Starting and Stopping the Container

#### Start Container (Detached Mode)
```bash
# Start container in background
docker compose up -d

# Verify it's running
docker compose ps
```

#### Stop Container
```bash
# Stop the container
docker compose down

# Stop and remove volumes
docker compose down -v
```

#### Restart Container
```bash
docker compose restart
```

### Accessing the Container

#### Open Main Terminal
```bash
# Enter the running container
docker exec -it px4-ros2-dev bash
```

#### Open Additional Terminals
```bash
# Open as many terminals as you need (in new terminal windows)
docker exec -it px4-ros2-dev bash
```

**Pro Tip**: Use a terminal multiplexer like `tmux` inside the container:
```bash
# Inside container
apt-get update && apt-get install -y tmux
tmux
# Now use Ctrl+b then c to create new windows
# Ctrl+b then n/p to navigate between windows
```

### Container Status and Logs

#### Check Container Status
```bash
# List all containers
docker ps -a

# Check specific container
docker compose ps
```

#### View Container Logs
```bash
# View logs
docker compose logs

# Follow logs in real-time
docker compose logs -f

# View last 100 lines
docker compose logs --tail=100
```

### File Operations

#### Copy Files to Container
```bash
# Copy from host to container
docker cp /path/on/host/file.txt px4-ros2-dev:/root/destination/

# Copy directory
docker cp /path/on/host/mydir px4-ros2-dev:/root/destination/
```

#### Copy Files from Container
```bash
# Copy from container to host
docker cp px4-ros2-dev:/root/source/file.txt /path/on/host/

# Copy directory
docker cp px4-ros2-dev:/root/source/mydir /path/on/host/
```

#### Using Mounted Workspace
The `./workspace` directory is mounted to `/root/workspace` in the container:
```bash
# On host: create files in ./workspace
echo "Hello" > workspace/test.txt

# Inside container: access immediately
cat /root/workspace/test.txt
```

### Cleaning Up

#### Remove Stopped Containers
```bash
# Remove stopped containers
docker container prune

# Remove with confirmation
docker container prune -f
```

#### Remove Unused Images
```bash
# Remove dangling images
docker image prune

# Remove all unused images
docker image prune -a
```

#### Complete Cleanup
```bash
# Remove everything (containers, images, volumes, networks)
docker system prune -a --volumes

# Free up space (will ask for confirmation)
docker system prune -a
```

#### Remove This Specific Setup
```bash
# Stop and remove container
docker compose down

# Remove the image
docker rmi px4-ros2-gazebo:latest

# Remove all related images
docker images | grep px4-ros2 | awk '{print $3}' | xargs docker rmi
```

---

## Running the PX4 Example

This example demonstrates the PX4 ROS2 Control Interface with a custom flight mode.

### Step 1: Start the Container

```bash
# From your project directory
cd ~/px4-ros2-docker

# Start container
docker compose up -d

# Verify it's running
docker compose ps
```

### Step 2: Open Multiple Terminals

Open 4 separate terminal windows/tabs on your host machine:

**Terminal 1**: PX4 SITL with Gazebo
```bash
docker exec -it px4-ros2-dev bash
```

**Terminal 2**: Micro-XRCE-DDS Agent
```bash
docker exec -it px4-ros2-dev bash
```

**Terminal 3**: ROS2 Example Mode
```bash
docker exec -it px4-ros2-dev bash
```

**Terminal 4**: PX4 Console Commands
```bash
docker exec -it px4-ros2-dev bash
```

### Step 3: Start Each Component

#### Terminal 1: Launch PX4 + Gazebo
```bash
cd /root/PX4-Autopilot
make px4_sitl gz_x500
```

**Expected Output**:
- Gazebo window opens showing a quadcopter
- Console shows: `INFO [commander] Ready for takeoff!`

**Wait for this to fully start before proceeding!**

#### Terminal 2: Start DDS Agent
```bash
MicroXRCEAgent udp4 -p 8888
```

**Expected Output**:
```
[1234567890.123456] info     | UDPv4AgentLinux.cpp | init | running...
[1234567890.123456] info     | Root.cpp           | set_verbose_level | logger verbosity set to 4
```

#### Terminal 3: Run Example Mode
```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 run example_mode_manual_cpp example_mode_manual
```

**Expected Output**:
```
[DEBUG] [example_mode_manual]: Checking message compatibility...
[DEBUG] [example_mode_manual]: Registering 'My Manual Mode'
[DEBUG] [example_mode_manual]: Got RegisterExtComponentReply
```

#### Terminal 4: Verify Mode Registration
```bash
cd /root/PX4-Autopilot
# Access PX4 console (pxh prompt should already be visible)
commander status
```

**Expected Output**:
```
INFO [commander] Disarmed
INFO [commander] navigation mode: Position
INFO [commander] External Mode 1: nav_state: 23, name: My Manual Mode
```

### Step 4: Test the Mode

1. **Start QGroundControl** on your host machine:
   ```bash
   # Download if you don't have it
   wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
   chmod +x QGroundControl.AppImage
   ./QGroundControl.AppImage
   ```

2. **Select the Mode**: In QGC, you should see "My Manual Mode" in the flight mode dropdown

3. **Arm and Test**: 
   - Select "My Manual Mode"
   - Ensure you have a joystick connected or use QGC's virtual joystick
   - Arm the vehicle
   - The mode should activate

---

## Common Workflows

### Daily Development Workflow

```bash
# 1. Start your day
cd ~/px4-ros2-docker
docker compose up -d

# 2. Open terminal
docker exec -it px4-ros2-dev bash

# 3. Do your work...

# 4. End of day - stop container
docker compose down
```

### Building Your Own Mode

```bash
# 1. Create your package in the workspace
cd ~/px4-ros2-docker/workspace
mkdir -p my_custom_mode/src

# 2. Enter container
docker exec -it px4-ros2-dev bash

# 3. Link to ROS workspace
cd /root/ros2_ws/src
ln -s /root/workspace/my_custom_mode .

# 4. Build
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_custom_mode

# 5. Source and run
source install/setup.bash
ros2 run my_custom_mode my_node
```

### Updating PX4

```bash
# Inside container
cd /root/PX4-Autopilot
git pull
git submodule update --init --recursive
make clean
make px4_sitl gz_x500
```

### Updating ROS2 Packages

```bash
# Inside container
cd /root/ros2_ws/src

# Update px4_msgs
cd px4_msgs
git pull

# Update interface library
cd ../px4-ros2-interface-lib
git pull
git submodule update --init --recursive

# Rebuild
cd /root/ros2_ws
colcon build
```

### Running Tests

```bash
# Inside container
cd /root/ros2_ws

# Run tests for a specific package
colcon test --packages-select px4_msgs

# Run all tests
colcon test

# View test results
colcon test-result --verbose
```

---

## Troubleshooting

### Container Won't Start

**Problem**: `docker compose up -d` fails

**Solutions**:
```bash
# Check logs
docker compose logs

# Check if port is in use
sudo netstat -tulpn | grep 8888

# Remove old containers
docker compose down
docker container prune

# Try again
docker compose up -d
```

### Can't See Gazebo GUI

**Problem**: Gazebo opens but shows blank screen or doesn't open

**Solutions**:
```bash
# On host, ensure X11 is allowed
xhost +local:docker

# Check DISPLAY variable inside container
docker exec -it px4-ros2-dev bash
echo $DISPLAY  # Should show :0 or :1

# If blank, try setting it
export DISPLAY=:0

# Test with simple GUI app
apt-get update && apt-get install -y x11-apps
xeyes  # Should show eyes following cursor
```

### Build Fails with "Out of Space"

**Problem**: Docker build fails with disk space error

**Solutions**:
```bash
# Clean up Docker
docker system prune -a --volumes

# Check disk space
df -h

# Remove old images
docker images
docker rmi <image_id>
```

### MicroXRCEAgent Connection Issues

**Problem**: Agent shows no clients connecting

**Solutions**:
```bash
# 1. Ensure PX4 is running first
# 2. Check if port is available
netstat -an | grep 8888

# 3. Restart agent with verbose logging
MicroXRCEAgent udp4 -p 8888 -v6

# 4. In PX4 console, check uXRCE-DDS status
cd /root/PX4-Autopilot
# In pxh console:
uxrce_dds_client status
```

### ROS2 Mode Not Showing in QGC

**Problem**: Custom mode doesn't appear in QGroundControl

**Solutions**:
```bash
# 1. Verify mode is registered
commander status  # In PX4 console

# 2. Ensure you're using QGC Daily (not stable)
# Download from: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

# 3. Restart QGC

# 4. Check ROS2 node is running
ros2 node list  # Should show your mode node
```

### Permission Denied Errors

**Problem**: Can't write files or execute commands

**Solutions**:
```bash
# Ensure you're in docker group
groups  # Should show 'docker'

# If not, add yourself
sudo usermod -aG docker $USER
# Log out and back in

# Inside container, everything runs as root, so no permission issues
```

### Container Keeps Restarting

**Problem**: Container starts then immediately stops

**Solutions**:
```bash
# Check what's happening
docker compose logs

# Run container interactively to see error
docker compose run --rm px4-ros2 bash

# Check entrypoint script
docker exec -it px4-ros2-dev cat /root/entrypoint.sh
```

---

## Development Tips

### Using tmux for Multiple Terminals

Instead of opening many terminal windows, use `tmux` inside the container:

```bash
# Install tmux (only needed once)
docker exec -it px4-ros2-dev bash
apt-get update && apt-get install -y tmux

# Start tmux
tmux

# Key bindings:
# Ctrl+b then c  : Create new window
# Ctrl+b then n  : Next window
# Ctrl+b then p  : Previous window
# Ctrl+b then 0-9: Switch to window number
# Ctrl+b then "  : Split horizontally
# Ctrl+b then %  : Split vertically
# Ctrl+b then arrow: Navigate between panes
# Ctrl+b then d  : Detach (tmux keeps running)
# tmux attach    : Reattach to session
```

### Saving Your Container State

If you've made changes inside the container you want to keep:

```bash
# Commit container to new image
docker commit px4-ros2-dev px4-ros2-gazebo:custom

# Update compose.yaml to use new image
# Change: image: px4-ros2-gazebo:latest
# To:     image: px4-ros2-gazebo:custom
```

### Accessing Container from Other Machines

If you want to access the container from another computer on your network:

```bash
# Inside container, install SSH server
apt-get update && apt-get install -y openssh-server
service ssh start

# Set root password
passwd

# From another machine
ssh root@<your-machine-ip>
```

### Backing Up Your Work

```bash
# Backup workspace
tar -czf workspace-backup-$(date +%Y%m%d).tar.gz workspace/

# Backup entire container
docker export px4-ros2-dev > px4-ros2-backup.tar

# Restore container
docker import px4-ros2-backup.tar px4-ros2-gazebo:restored
```

### Quick Reference Card

```bash
# Start working
docker compose up -d && docker exec -it px4-ros2-dev bash

# Stop working  
docker compose down

# New terminal
docker exec -it px4-ros2-dev bash

# View logs
docker compose logs -f

# Rebuild
docker compose build

# Clean restart
docker compose down && docker compose up -d

# Full cleanup
docker compose down && docker system prune -a
```

---

## Additional Resources

- **PX4 Documentation**: https://docs.px4.io/
- **ROS2 Humble Documentation**: https://docs.ros.org/en/humble/
- **Gazebo Documentation**: https://gazebosim.org/docs/harmonic
- **PX4 ROS2 Interface Library**: https://github.com/Auterion/px4-ros2-interface-lib
- **Docker Documentation**: https://docs.docker.com/

---

## Getting Help

If you encounter issues:

1. Check this README's troubleshooting section
2. Check container logs: `docker compose logs`
3. Search PX4 forum: https://discuss.px4.io/
4. Check GitHub issues: https://github.com/PX4/PX4-Autopilot/issues

---

## License

This Docker setup is provided as-is. PX4, ROS2, and Gazebo have their own respective licenses.

---

*Last Updated: October 2025*
