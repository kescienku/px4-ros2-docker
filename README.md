# PX4 v1.16 + ROS2 Humble + Gazebo Harmonic Docker Environment

[![PX4](https://img.shields.io/badge/PX4-v1.16.0-blue)](https://github.com/PX4/PX4-Autopilot)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?logo=docker)](https://www.docker.com/)

**Complete, working Docker environment for PX4 SITL development with ROS2 integration and Gazebo Harmonic simulation.**

No more version mismatches, dependency hell, or "it works on my machine" issues. Everything just works! 🚀

## ✨ Features

- 🎯 **Version-matched components** - PX4, ROS2, and Gazebo versions guaranteed compatible
- 🔧 **Micro-XRCE-DDS Agent** - Pre-installed for seamless PX4-ROS2 communication
- 🎮 **Custom Flight Modes** - px4-ros2-interface-lib included for advanced control
- 📚 **Comprehensive Docs** - Step-by-step guides for everything
- 🐛 **Troubleshooting Guide** - Solutions to common errors documented
- ⚡ **Fast Rebuilds** - Docker layer caching for quick iterations

## 🎬 Quick Start

```bash
# Clone and build
git clone https://github.com/kescienku/px4-ros2-docker.git
cd px4-ros2-docker
chmod +x build.sh run.sh
./build.sh  # First time: 20-30 min

# Run
./run.sh

# Inside container - Start PX4 + Gazebo
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

That's it! Gazebo GUI opens with a drone, ROS2 topics flowing, ready to develop.

---

**📚 Additional Guides:**
- **CONTAINER_USAGE.md** - Advanced container management, mounting code, multiple terminals
- **ERRORS_REFERENCE.md** - Complete error reference and troubleshooting
  - Chapter 1: Docker build errors
  - Chapter 2: PX4 ROS2 Control Interface setup

## What's Included

- **PX4 v1.16 Stable** (source code + pre-built SITL binary)
- **ROS2 Humble** (full desktop installation)
- **Gazebo Harmonic** (with GUI support)
- **px4_msgs** (release/1.16 - matching PX4 version)
- **px4_ros_com** (release/1.16 - for PX4-ROS2 bridge)
- **px4-ros2-interface-lib** (release/1.16 - for custom flight modes)
- **ROS2 workspace** at `/home/px4user/ros2_ws` for your development

## Quick Start

### 1. Build the Docker Image

```bash
chmod +x build.sh
./build.sh
```

**First build**: Takes **20-30 minutes** (downloading and compiling everything)

**Subsequent builds**: Takes **seconds to minutes** depending on what changed
- No changes → Uses cache → ⚡ Seconds
- Added packages → Rebuilds from that point → Minutes  
- Changed early lines → Full rebuild → 20-30 minutes

This is **normal Docker behavior** using layer caching for speed!

**Force full rebuild** (if needed):
```bash
docker build --no-cache -t px4_ros2_gz:latest .
```

### 2. Run the Container

```bash
chmod +x run.sh
./run.sh
```

You'll be inside the container as `px4user` in the `ros2_ws` directory.

### 3. Verify Installation

Inside the container:

```bash
chmod +x test_setup.sh
./test_setup.sh
```

## Running PX4 SITL with Gazebo

### Method 1: Automatic Agent (Recommended)

**Terminal 1 (PX4 + Gazebo + Agent):**

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

This automatically:
- Starts PX4 SITL
- Launches Gazebo Harmonic GUI with X500 drone
- **Starts uXRCE-DDS agent automatically** on port 8888

### Method 2: Manual Agent Control

If you need manual control over the agent:

**Terminal 1 (Agent only):**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 (PX4+ Gazebo):**
```bash
cd ~/PX4-Autopilot
PX4_UXRCE_DDS_AG_IP=0.0.0.0 make px4_sitl gz_x500
```

### Terminal 3 (ROS2):

### Terminal 3 (ROS2):

Open a new terminal and enter the same container:

```bash
docker exec -it px4_dev bash
```

Then:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

You should see PX4 topics like:
- `/fmu/in/...`
- `/fmu/out/...`

### Verifying Agent Connection

Check if agent is working:
```bash
ros2 topic hz /fmu/out/vehicle_status
```

Should show ~1 Hz updates. If no data, check:
```bash
# In PX4 terminal, type:
uxrce_dds_client status
```

## Developing ROS2 Packages

Your ROS2 workspace is at: `/home/px4user/ros2_ws`

### Create a new package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_px4_package
```

### Build:

```bash
cd ~/ros2_ws
colcon build --packages-select my_px4_package
source install/setup.bash
```

## Container Structure

```
/home/px4user/
├── PX4-Autopilot/          # PX4 v1.16 source + built SITL
├── ros2_ws/                # Your ROS2 workspace
│   ├── src/
│   │   ├── px4_msgs/       # PX4 message definitions
│   │   ├── px4_ros_com/    # PX4-ROS2 bridge
│   │   ├── px4-ros2-interface-lib/  # Control interface library
│   │   └── (your packages) # Add your packages here
│   ├── build/
│   └── install/
```

## Opening Multiple Terminals

To open additional terminals in the running container:

```bash
docker exec -it px4_dev bash
```

## Stopping the Container

Just type `exit` or press `Ctrl+D`. The container will be removed automatically (--rm flag).

## Re-entering Container

Since we use `--rm`, the container is deleted on exit. This is intentional to keep things clean.

If you want to persist your ROS2 workspace, modify `run.sh` to:
1. Remove the `--rm` flag
2. Add: `-v ~/px4_workspace:/home/px4user/ros2_ws/src`

## Troubleshooting

**See `ERRORS_REFERENCE.md` for a complete list of common errors and solutions.**

### Gazebo GUI not showing:

```bash
# On host machine:
echo $DISPLAY
xhost +local:docker
```

### PX4 messages not found in ROS2:

```bash
source ~/ros2_ws/install/setup.bash
ros2 interface list | grep px4_msgs
```

### Need to rebuild PX4:

```bash
cd ~/PX4-Autopilot
make clean
make px4_sitl gz_x500
```

## Version Details

- **Ubuntu**: 22.04
- **ROS2**: Humble
- **PX4**: v1.16.0
- **Gazebo**: Harmonic
- **Micro-XRCE-DDS Agent**: Latest (for PX4-ROS2 communication)
- **px4_msgs**: release/1.16 (exact match with PX4)
- **px4_ros_com**: release/1.16
- **px4-ros2-interface-lib**: release/1.16 (for custom flight modes)

## What You Can Do

✅ Develop ROS2 packages inside container  
✅ Run PX4 SITL simulations  
✅ View Gazebo GUI on host display  
✅ PX4-ROS2 communication via DDS  
✅ Modify PX4 source if needed  
✅ Test custom flight controllers  

## Notes

- The workspace is inside the container (not mounted from host)
- If you need persistence, modify the `run.sh` script to add volume mounts
- Container runs as non-root user `px4user` with sudo access
- PX4 is already built, no need to rebuild unless you modify source

## Example: Simple ROS2 Subscriber

Create a simple subscriber to PX4 vehicle status:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python px4_listener
cd px4_listener/px4_listener

# Edit listener.py with your code
# Then build and run:
cd ~/ros2_ws
colcon build --packages-select px4_listener
source install/setup.bash
ros2 run px4_listener listener
```

## Example: Custom Flight Mode with Control Interface

The container includes **px4-ros2-interface-lib** for creating custom flight modes:

```bash
# Terminal 1: Start PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2: Run example mode
docker exec -it px4_dev bash
source ~/ros2_ws/install/setup.bash
ros2 run example_mode_manual_cpp example_mode_manual
```

See [PX4 ROS 2 Control Interface docs](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface) for more examples.

**Note:** See `ERRORS_REFERENCE.md` Chapter 2 for troubleshooting control interface issues.

---

**That's it! You now have a fully working PX4-ROS2-Gazebo development environment.**
