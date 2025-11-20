# PX4-ROS2-Gazebo Docker Setup - Error Reference

This document logs all errors encountered during setup and their solutions for future reference.

---

## Error 1: Package 'ros-humble-ros-gzharmonic' Not Found

**Error Message:**
```
E: Unable to locate package ros-humble-ros-gzharmonic
```

**Cause:**  
The package `ros-humble-ros-gzharmonic` doesn't exist in ROS2 Humble repositories. The correct package name is `ros-humble-ros-gz`.

**Solution:**  
- Remove `ros-humble-ros-gzharmonic` from ROS2 installation step
- Install Gazebo Harmonic first from OSRFoundation repos
- Then install `ros-humble-ros-gz` separately after Gazebo is installed
- Also install `libgz-sim8-dev` for development headers

**Fixed in:** Dockerfile lines 28-42

---

## Error 2: Remote Branch 'v1.16.0-stable' Not Found

**Error Message:**
```
fatal: Remote branch v1.16.0-stable not found in upstream origin
```

**Cause:**  
The branch name `v1.16.0-stable` doesn't exist in PX4 repository. PX4 uses tag names like `v1.16.0` for stable releases, not branch names with `-stable` suffix.

**Solution:**  
Use the correct tag name: `v1.16.0` (without `-stable`)
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive --branch v1.16.0
```

**Note:** PX4 release tags follow the pattern: `v1.16.0`, `v1.16.0-rc1`, `v1.16.0-beta1`, etc.

**Fixed in:** Dockerfile line 58

---

## Error 3: Build Hangs at PX4 Compilation Step

**Error Message:**
```
[1047/1048] Linking CXX executable bin/px4
pxh> 
[output clipped, log limit 2MiB reached]
CANCELED
```

**Cause:**  
The Dockerfile was using `make px4_sitl gz_x500` which not only **builds** PX4 but also **runs** it interactively. The `pxh>` prompt means PX4 is waiting for user input, causing Docker build to hang indefinitely.

**Solution:**  
Use build-only command without target specification:
```dockerfile
make -j$(nproc) px4_sitl_default
```

This compiles PX4 without starting the simulator. You can run it manually later inside the container with `make px4_sitl gz_x500`.

**Note:** Do NOT use `gz-sim` as a target - it's not a valid ninja build target.

**Fixed in:** Dockerfile line 73

---

## Error 4: Unknown Target 'gz-sim'

**Error Message:**
```
ninja: error: unknown target 'gz-sim'
make: *** [Makefile:227: px4_sitl_default] Error 1
```

**Cause:**  
`gz-sim` is not a valid build target for ninja. The correct approach is to just build the default SITL target.

**Solution:**  
Build only the SITL default configuration:
```dockerfile
make -j$(nproc) px4_sitl_default
```

The Gazebo models and plugins are built as part of the default SITL target automatically.

**Fixed in:** Dockerfile line 76

---

## Error 5: Gazebo Simulation Dependencies Not Found (Your Original Error)

**Error Message:**
```
ERROR: Gazebo simulation dependencies not found!
- For installation instructions, see: https://gazebosim.org/docs/harmonic/install_ubuntu/
ninja: build stopped: subcommand failed.
make: *** [Makefile:232: px4_sitl] Error 1
```

**Cause:**  
PX4 build tried to compile Gazebo support before Gazebo Harmonic was installed on the system.

**Solution:**  
Install Gazebo Harmonic BEFORE building PX4. The installation order must be:
1. ROS2 Humble
2. Gazebo Harmonic + dev libraries
3. ROS-Gazebo bridge
4. PX4 dependencies
5. Build PX4 SITL

**Fixed in:** Dockerfile line order (Gazebo installed before PX4 clone/build)

---

## Error 6: PX4 Messages Version Mismatch

**Error Message:**
```
CMake Error: px4_msgs interfaces do not match PX4 definitions
Could not find a package configuration file provided by "px4_msgs"
```

**Cause:**  
Using wrong branch/version of `px4_msgs` that doesn't match PX4 firmware version. Main/master branches often don't match stable releases.

**Solution:**  
Always use the exact matching release branch:
- For PX4 v1.16: Use `px4_msgs` branch `release/1.16`
- For PX4 v1.16: Use `px4_ros_com` branch `release/1.16`

**Fixed in:** Dockerfile lines 77-82 (explicit branch specification)

---

## Error 7: Ninja Build Errors

**Error Message:**
```
ninja: error: loading 'build.ninja': No such file or directory
```

**Cause:**  
PX4 build system not properly initialized, or wrong Python dependencies installed.

**Solution:**  
- Ensure all PX4 Python dependencies are installed (especially `empy==3.3.4`)
- Run PX4's setup script: `bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools`
- Clean and rebuild if needed: `make clean && make px4_sitl gz_x500`

**Fixed in:** Dockerfile lines 61-68 (proper Python deps + setup script)

---

## Error 8: X11 Display Not Working (Gazebo GUI Blank/Doesn't Show)

**Error Message:**
```
cannot connect to X server
```

**Cause:**  
Docker doesn't have permission to access host X11 display server.

**Solution:**  
Before running container:
```bash
xhost +local:docker
```

After exiting:
```bash
xhost -local:docker
```

**Fixed in:** run.sh script (includes xhost commands)

---

## Error 9: Missing Gazebo Models

**Error Message:**
```
[Wrn] [ResourceSpawner.cc:XXX] Resource path not found
```

**Cause:**  
Gazebo can't find PX4 custom models (x500, etc.)

**Solution:**  
Set environment variable:
```bash
export GZ_SIM_RESOURCE_PATH=/home/px4user/PX4-Autopilot/Tools/simulation/gz/models
```

**Fixed in:** Dockerfile line 94 (added to .bashrc)

---

## Error 10: rosdep Init Fails on Second Run

**Error Message:**
```
ERROR: default sources list file already exists
```

**Cause:**  
`rosdep init` was already run before (creates system-wide config).

**Solution:**  
Use `|| true` to ignore error if already initialized:
```bash
sudo rosdep init || true
rosdep update
```

**Fixed in:** Dockerfile line 72

---

## Error 11: Permission Denied When Building PX4

**Error Message:**
```
permission denied: /home/px4user/PX4-Autopilot/build/
```

**Cause:**  
Building as root user, then switching to px4user causes permission issues.

**Solution:**  
Always build PX4 as the px4user (non-root):
```dockerfile
USER px4user
WORKDIR /home/px4user/PX4-Autopilot
RUN make px4_sitl gz_x500
```

**Fixed in:** Dockerfile lines 70-73 (build as px4user)

---

## Error 12: uXRCE-DDS Agent Not Starting

**Error Message:**
```
Failed to create client
```

**Cause:**  
PX4's built-in microXRCE-DDS agent not starting automatically, or ROS2 trying to connect before it's ready.

**Solution:**  
When running `make px4_sitl gz_x500`, the agent starts automatically. Wait 5-10 seconds before trying ROS2 topic list.

Alternative - manual start:
```bash
MicroXRCEAgent udp4 -p 8888
```

**Note:** In PX4 v1.16, agent starts automatically with SITL.

---

## Error 13: Package Not Found After Building

**Error Message:**
```
Package 'my_package' not found
```

**Cause:**  
Forgot to source the workspace after building.

**Solution:**  
Always source after colcon build:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

**Fixed in:** Dockerfile line 94 (auto-source in .bashrc)

---

## Quick Troubleshooting Checklist

If something doesn't work, check in this order:

1. **Is ROS2 sourced?**
   ```bash
   echo $ROS_DISTRO  # Should show: humble
   ```

2. **Is Gazebo installed?**
   ```bash
   gz sim --version  # Should show: Harmonic
   ```

3. **Is PX4 built?**
   ```bash
   ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
   ```

4. **Are px4_msgs available?**
   ```bash
   ros2 interface list | grep px4_msgs
   ```

5. **Is display working?**
   ```bash
   echo $DISPLAY  # Should show something like :0 or :1
   xhost  # Should show: access control disabled
   ```

---

## Prevention Tips

1. **Always match versions:** PX4 v1.16 → px4_msgs release/1.16 → px4_ros_com release/1.16
2. **Install order matters:** ROS2 → Gazebo → PX4 dependencies → Build PX4
3. **Source workspaces:** Add to .bashrc or source manually after each build
4. **Clean builds:** If weird errors occur, try `make clean` before rebuilding
5. **Check logs:** PX4 and Gazebo logs are in `~/.ros/log/` and `~/PX4-Autopilot/build/`

---

**Last Updated:** Based on setup with Docker 29.0.0, Ubuntu 22.04 host  
**Working Configuration:** PX4 v1.16.0 + ROS2 Humble + Gazebo Harmonic
