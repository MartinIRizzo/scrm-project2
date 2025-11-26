# Troubleshooting Guide

Common issues and solutions for the MUSI-nav2 learning environment.

---

## Docker Issues

### "Cannot connect to the Docker daemon" or "Is the docker daemon running?"

This error means Docker is not running or your user doesn't have permission to access it.

#### WSL2 Solution

1. Ensure Docker Desktop is running on Windows
2. Verify WSL integration is enabled:
   - Open Docker Desktop → Settings → Resources → WSL Integration
   - Enable for your Ubuntu distribution
3. Restart Docker Desktop if needed
4. Test in WSL: `docker ps`

#### Linux Solution

```bash
# Start Docker service
sudo systemctl start docker
sudo systemctl enable docker

# Add user to docker group if not already done
sudo usermod -aG docker $USER
# Logout and login again for group changes to take effect
```

#### macOS Solution

1. Ensure Docker Desktop is running (check menu bar icon)
2. Restart Docker Desktop if needed
3. Test: `docker ps`

---

### "Permission denied" when running scripts

The shell scripts need execute permissions.

**Solution**:
```bash
chmod +x build.sh run.sh stop.sh
```

---

### "docker: command not found"

Docker is not installed on your system.

**Solution**: Follow the installation instructions for your platform in [Installation Guide](installation.md).

Quick install for Linux:
```bash
sudo apt update
sudo apt install -y docker.io docker-compose
sudo usermod -aG docker $USER
# Logout and login again
```

---

### Container starts but immediately exits

The container may be failing to start due to configuration or resource issues.

**Solution**: Check the container logs:
```bash
docker logs nav2_workspace
```

Common causes:
- Insufficient memory (need at least 2 GB available)
- Port conflicts (unlikely with host networking)
- Corrupted image (rebuild with `docker-compose down && ./build.sh`)

---

### Out of disk space during build

Docker images and layers can consume significant disk space.

**Solution**:
```bash
# Check available space
df -h

# Clean Docker system (removes all unused images/containers)
docker system prune -a

# If still not enough, clean apt cache inside container
docker exec nav2_workspace bash -c "apt clean"
```

**Warning**: `docker system prune -a` removes all unused Docker images and containers!

---

## GUI and X11 Issues

### GUI applications don't display (Gazebo/RViz2)

This is usually an X11 forwarding issue. Solutions vary by platform.

#### Linux Solution

```bash
# Enable X11 forwarding
xhost +local:docker
export DISPLAY=:0

# Restart the container
docker-compose restart nav2_dev
```

**Check X11 socket** exists:
```bash
ls -la /tmp/.X11-unix/
# Should show X0 socket
```

#### WSL2 Solution

**Windows 11 (WSLg)**: Should work automatically. If not:
```bash
# Add to ~/.bashrc
export DISPLAY=:0
source ~/.bashrc
```

**Windows 10 (VcXsrv/X410)**:
```bash
# Add to ~/.bashrc
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}):0
source ~/.bashrc

# Ensure X server is running on Windows
# Launch VcXsrv with: Multiple windows, Start no client, Disable access control
```

#### macOS Solution

```bash
# Ensure XQuartz is running
# Enable X11 forwarding
xhost +localhost
export DISPLAY=host.docker.internal:0

# Restart container
docker-compose restart nav2_dev
```

---

### Gazebo shows "not responding" dialog

This is **normal behavior** on first launch while Gazebo compiles shaders.

**Solution**: Click **"Wait"** and give it 30-60 seconds. Subsequent launches will be faster.

---

## Gazebo and Simulation Issues

### Gazebo window is black or frozen

This usually happens when:
1. First launch (shader compilation)
2. Insufficient GPU resources
3. X11 forwarding not working

**Solutions**:

1. **Wait for shader compilation** (first launch only): Give it 60 seconds
2. **Check X11 forwarding**: Follow GUI troubleshooting above
3. **Try headless mode**: Use Gazebo without GUI:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false
   # Use RViz2 for visualization instead
   rviz2
   ```
4. **Reduce resource usage**: See [Resource Optimization Guide](resource_optimization.md)

---

### Robot doesn't move in Gazebo

Check if simulation is running and robot is spawned correctly.

**Diagnosis**:
```bash
# Check if Gazebo topics are active
ros2 topic list | grep gazebo

# Check if robot topics exist
ros2 topic list | grep cmd_vel
ros2 topic list | grep odom

# Try publishing velocity command manually
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

**Common causes**:
- Gazebo is paused (click Play button in Gazebo GUI)
- Physics not running (check Gazebo console for errors)
- Robot model failed to spawn (check `docker logs nav2_workspace`)

---

### Gazebo crashes or shows "Unable to create the rendering window"

This is typically a graphics driver or X11 issue.

**Solutions**:

1. **Use headless mode** (no GUI):
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false
   ```

2. **Update graphics drivers** (Linux):
   ```bash
   # Check current driver
   lspci -k | grep -A 2 -E "(VGA|3D)"

   # Update drivers (Ubuntu)
   sudo ubuntu-drivers autoinstall
   ```

3. **Use software rendering** (slower but more compatible):
   ```bash
   # Inside container, add to ~/.bashrc
   export LIBGL_ALWAYS_SOFTWARE=1
   ```

---

## ROS2 Communication Issues

### ROS2 nodes can't communicate between terminals

Nodes must be in the same ROS domain and network.

**Solution**: The container uses host networking, which should work automatically. Verify:

```bash
# Check ROS_DOMAIN_ID is consistent across terminals
echo $ROS_DOMAIN_ID  # Should be 0

# Test communication
# Terminal 1:
ros2 topic pub /test_topic std_msgs/String "data: hello"

# Terminal 2:
ros2 topic echo /test_topic
# Should show: data: hello
```

If still not working:
```bash
# Check container networking mode
docker inspect nav2_workspace | grep NetworkMode
# Should show: "host"

# Verify firewall allows multicast (Linux)
sudo ufw allow proto udp from any to 224.0.0.0/4
```

---

### RViz2 error "frame [map] does not exist"

This happens when the map frame hasn't been published yet (before navigation starts).

**Solution**: Change the Fixed Frame in RViz2:
1. Left panel → Global Options → Fixed Frame
2. Change from "map" to **"odom"**

After starting navigation with a map, you can change it back to "map".

---

### "No executable found" error

ROS2 can't find the package or node you're trying to run.

**Diagnosis**:
```bash
# Check if package is installed
ros2 pkg list | grep <package_name>

# Check package executables
ros2 pkg executables <package_name>

# Source workspace if using custom packages
source /ros2_ws/install/setup.bash
```

**Solutions**:
- Ensure package is installed (`ros2 pkg list`)
- Source workspace if custom package (`source install/setup.bash`)
- Rebuild workspace if code changed (`cd /ros2_ws && colcon build`)

---

### Topics not showing up in `ros2 topic list`

This usually means nodes haven't started or are in a different ROS domain.

**Diagnosis**:
```bash
# Check if nodes are running
ros2 node list

# Check ROS domain ID
echo $ROS_DOMAIN_ID

# Check if simulation is running
ps aux | grep gazebo
```

**Solutions**:
1. Ensure simulation is launched: `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
2. Wait for Gazebo to fully start (30-60 seconds)
3. Check `docker logs nav2_workspace` for errors

---

## Performance Issues

### High CPU or memory usage

Gazebo and RViz2 are resource-intensive applications.

**Quick fixes**:

1. **Use headless Gazebo**:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false
   ```
   Saves ~60% CPU and ~500 MB RAM

2. **Use lighter robot model**:
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

3. **Use empty world**:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_empty_world.launch.py
   ```

For comprehensive performance optimization, see [Resource Optimization Guide](resource_optimization.md).

---

### Slow simulation / Low FPS

This indicates system resources are strained.

**Diagnosis**:
```bash
# Monitor container resources
docker stats nav2_workspace

# Inside container, check processes
docker exec -it nav2_workspace bash
top  # Press 'q' to quit
```

**Solutions**:
1. Close other resource-intensive applications
2. Use headless Gazebo (see above)
3. Reduce physics update rate (create `~/.gazebo/gui.ini`):
   ```bash
   docker exec -it nav2_workspace bash
   mkdir -p ~/.gazebo
   echo "[physics]
   max_step_size = 0.01
   real_time_update_rate = 100" > ~/.gazebo/gui.ini
   ```
4. Limit Docker resources in `docker-compose.yml`:
   ```yaml
   deploy:
     resources:
       limits:
         cpus: '4.0'
         memory: 4G
   ```

---

## Build Issues

### Build fails with "No space left on device"

Insufficient disk space for Docker layers.

**Solution**:
```bash
# Check available space
df -h

# Clean Docker system
docker system prune -a

# Clean apt cache
sudo apt clean
```

---

### Build fails with package download errors

Network connectivity or repository issues.

**Solutions**:

1. **Check internet connection**:
   ```bash
   ping google.com
   ```

2. **Retry the build**:
   ```bash
   ./build.sh
   # Docker caches successful layers, so it will resume
   ```

3. **Update package lists** (if using apt in Dockerfile):
   ```bash
   # Rebuild without cache
   docker-compose build --no-cache
   ```

---

## Getting More Help

If you're still experiencing issues:

1. **Check Docker logs**:
   ```bash
   docker logs nav2_workspace
   ```

2. **Check ROS2 logs** (inside container):
   ```bash
   cd ~/.ros/log
   ls -ltr  # Shows most recent log directories
   cat <latest_directory>/<node_name>/stdout.log
   ```

3. **Verify environment variables**:
   ```bash
   env | grep -E "ROS|GAZEBO|DISPLAY|TURTLEBOT"
   ```

4. **Report issues**: https://github.com/ericguerrero/MUSI-nav2/issues

---

## Quick Diagnostic Commands

```bash
# System check
docker --version
docker ps
df -h

# Container check
docker logs nav2_workspace
docker stats nav2_workspace

# ROS2 check (inside container)
ros2 doctor
ros2 node list
ros2 topic list
echo $ROS_DOMAIN_ID

# Simulation check (inside container)
ps aux | grep gazebo
ros2 topic hz /scan  # Should show ~5-30 Hz
```

---

## Related Documentation

- [Installation Guide](installation.md) - Setup instructions
- [Resource Optimization Guide](resource_optimization.md) - Performance tuning
- [ROS2 Essentials](ros2_essentials.md) - ROS2 command reference
