# Container Management Guide

Complete guide for managing your PX4 ROS2 development container.

## Starting the Container

### Default Usage (Ephemeral)
```bash
./run.sh
```

This starts a **temporary** container:
- ✅ Automatic cleanup on exit
- ❌ All work inside container is **lost** on exit
- Use this for: Testing, running simulations

### Persistent Container (Recommended for Development)

Create `run_persistent.sh`:
```bash
#!/bin/bash

xhost +local:docker

docker run -it \
    --name px4_dev_persistent \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e QT_X11_NO_MITSHM=1 \
    px4_ros2_gz:latest

xhost -local:docker
```

**Note:** Removed `--rm` flag to keep container.

---

## Stopping and Restarting

### Stop Container (Keeps Data)
```bash
docker stop px4_dev
# or
docker stop px4_dev_persistent
```

Your work is **saved** inside the stopped container.

### Restart Stopped Container
```bash
docker start -i px4_dev_persistent
```

### List All Containers
```bash
docker ps -a
```

---

## Opening Multiple Terminals

### While Container is Running

**Method 1: Using docker exec (Recommended)**
```bash
# Terminal 2
docker exec -it px4_dev bash

# Terminal 3
docker exec -it px4_dev bash

# Terminal N...
docker exec -it px4_dev bash
```

**Method 2: Using tmux (inside container)**
```bash
# Install tmux first time
sudo apt update && sudo apt install -y tmux

# Start tmux session
tmux

# Split panes:
# Ctrl+b then "   (horizontal split)
# Ctrl+b then %   (vertical split)
# Ctrl+b then arrow keys (navigate)
# Ctrl+b then d   (detach session)
```

---

## Exiting the Container

### Exit Current Terminal Only
```bash
exit
# or press Ctrl+D
```

If using `--rm` (default `run.sh`):
- Last terminal exit → container is **deleted**

If using persistent mode:
- Last terminal exit → container **stops** but data is saved

### Force Stop Container
```bash
# From host machine
docker stop px4_dev
```

---

## Mounting Host Code into Container

### Option 1: Mount Specific Package
```bash
docker run -it --rm \
    --name px4_dev \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ~/my_ros_package:/home/px4user/ros2_ws/src/my_ros_package \
    -e QT_X11_NO_MITSHM=1 \
    px4_ros2_gz:latest
```

Now you can edit `~/my_ros_package` on your **host** machine with your favorite editor, and changes appear **instantly** inside the container!

### Option 2: Mount Entire Workspace
```bash
-v ~/my_ros2_workspace/src:/home/px4user/ros2_ws/src
```

**Warning:** This **replaces** the entire `src` folder, so px4_msgs, px4_ros_com, etc. must be in your host workspace.

### Option 3: Mount Additional Package Alongside Existing
```bash
-v ~/my_custom_pkg:/home/px4user/ros2_ws/src/my_custom_pkg
```

This adds your package without replacing existing ones. ✅ Recommended!

---

## Removing Containers

### Remove Stopped Container
```bash
docker rm px4_dev_persistent
```

### Remove Running Container (Force)
```bash
docker rm -f px4_dev
```

### Remove All Stopped Containers
```bash
docker container prune
```

---

## Customization Examples

### Example 1: Development Setup with Persistent Storage

Create `run_dev.sh`:
```bash
#!/bin/bash

xhost +local:docker

docker run -it \
    --name px4_dev \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ~/my_drone_code:/home/px4user/ros2_ws/src/my_drone_code \
    -e QT_X11_NO_MITSHM=1 \
    px4_ros2_gz:latest

xhost -local:docker
```

Now:
- Edit code on host with VSCode/Vim
- Build inside container: `cd ~/ros2_ws && colcon build`
- Code persists even if container is deleted
- Restart anytime: `docker start -i px4_dev`

### Example 2: Different Container Names for Different Projects

```bash
# Project A
docker run --name px4_project_a ... px4_ros2_gz:latest

# Project B  
docker run --name px4_project_b ... px4_ros2_gz:latest

# Switch between them
docker start -i px4_project_a
docker start -i px4_project_b
```

### Example 3: Shared /tmp for Fast Data Transfer

```bash
-v /tmp:/tmp
```

Now files in `/tmp` on host are accessible at `/tmp` in container.

---

## Common Workflows

### Workflow 1: Quick Testing (Default)
```bash
./run.sh                    # Start
# ... do your work ...
exit                        # Exit and auto-cleanup
```

### Workflow 2: Active Development
```bash
# Day 1
./run_persistent.sh         # Start
# ... work on code ...
exit                        # Stop (saves work)

# Day 2
docker start -i px4_dev_persistent  # Resume
# ... continue work ...
exit

# Day N
docker start -i px4_dev_persistent
```

### Workflow 3: Host + Container Development
```bash
# Start with mounted code
docker run -it --name px4_dev \
    -v ~/my_pkg:/home/px4user/ros2_ws/src/my_pkg \
    ... other flags ... \
    px4_ros2_gz:latest

# Terminal 1 (Container): Build and run
cd ~/ros2_ws
colcon build --packages-select my_pkg
ros2 run my_pkg my_node

# Terminal 2 (Host): Edit with IDE
code ~/my_pkg  # VSCode
# Changes appear instantly in container!

# Terminal 3 (Container): Test
docker exec -it px4_dev bash
ros2 topic echo /my_topic
```

---

## Troubleshooting

### Container Exits Immediately
```bash
# Check logs
docker logs px4_dev

# Try running with bash explicitly
docker run -it --rm px4_ros2_gz:latest /bin/bash
```

### "Name Already in Use"
```bash
# Remove old container
docker rm px4_dev

# Or use different name
docker run --name px4_dev_v2 ...
```

### Can't See Gazebo GUI
```bash
# On host
echo $DISPLAY        # Should show :0 or :1
xhost +local:docker  # Allow X11 access

# Restart container
```

### Changes Lost After Exit
- You're using `--rm` flag (ephemeral mode)
- Switch to persistent mode (remove `--rm`)
- Or mount your code from host

---

## Best Practices

1. **For Testing**: Use default `./run.sh` with `--rm`
2. **For Development**: Use persistent mode without `--rm`
3. **For Code**: Mount from host with `-v`
4. **For Collaboration**: Keep Dockerfile in git, share image
5. **For Multiple Projects**: Use different container names
6. **For Performance**: Mount only what you need
7. **For Safety**: Never mount `/` or `/home` entirely

---

## Quick Reference

| Action | Command |
|--------|---------|
| Start (temporary) | `./run.sh` |
| Start (persistent) | `docker run --name px4_dev ...` (no --rm) |
| Stop | `docker stop px4_dev` |
| Restart | `docker start -i px4_dev` |
| New terminal | `docker exec -it px4_dev bash` |
| Remove | `docker rm px4_dev` |
| Exit terminal | `exit` or `Ctrl+D` |
| List containers | `docker ps -a` |
| Mount code | `-v ~/code:/home/px4user/ros2_ws/src/code` |

---

**Remember**: Containers are **disposable**. Important code should always be on the host and mounted into the container!
