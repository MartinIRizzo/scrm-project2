# Resource Optimization Guide

Optimize the MUSI-nav2 environment for low-spec systems and reduce resource consumption.

---

## Overview

If your computer has limited resources (< 8 GB RAM or older CPU), you can significantly reduce resource consumption while maintaining core learning functionality.

**Target Systems**: Computers with 4-6 GB RAM, dual-core CPUs, or integrated graphics.

---

## Quick Comparison

| Configuration | CPU Usage | RAM Usage | Startup Time | Use Case |
|---------------|-----------|-----------|--------------|----------|
| **Full** (Waffle + GUI + World) | ~150% | ~2.5 GB | ~45s | Full-featured learning |
| **Headless** (GUI off) | ~40% | ~2.0 GB | ~20s | Background simulation |
| **Burger Model** | ~100% | ~2.2 GB | ~35s | Lighter robot |
| **Empty World** | ~80% | ~1.8 GB | ~25s | Basic testing |
| **Low-Spec** (Burger + Headless + Empty) | ~30% | ~1.2 GB | ~8s | Minimal systems |

---

## Optimization Options

### Option 1: Use Headless Gazebo (No GUI)

**Impact**: Saves ~60% CPU and ~500 MB RAM

Instead of full Gazebo, use the headless server and visualize in RViz2:

```bash
# Terminal 1: Launch headless Gazebo (no GUI)
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false

# Terminal 2: Use RViz2 for visualization (lighter than Gazebo GUI)
rviz2
```

**Benefits**:
- Gazebo GUI: ~150% CPU → Headless: ~40% CPU
- Visualize everything you need in RViz2
- Faster startup (~10 seconds vs ~45 seconds)
- All sensors and physics work identically

**When to use**:
- Systems with < 8 GB RAM
- Integrated graphics (Intel HD, etc.)
- Background simulation for data collection
- Focus on Nav2 learning, not world visualization

---

### Option 2: Use TurtleBot3 Burger (Lighter Model)

**Impact**: Saves ~30% CPU and ~300 MB RAM

The Burger model is lighter than Waffle (no camera, simpler structure):

```bash
# Set environment variable for Burger model
export TURTLEBOT3_MODEL=burger

# Launch with Burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Benefits**:
- ~30% faster simulation
- Burger still has LiDAR (sufficient for navigation learning)
- Smaller footprint, faster physics calculations

**Trade-offs**:
- No camera sensor (can't test vision-based navigation)
- Simpler appearance (but same navigation capabilities)

**When to use**:
- Learning basic navigation (SLAM, localization, path planning)
- Systems with limited CPU
- Faster iteration during development

---

### Option 3: Use Empty World

**Impact**: Saves ~50% CPU

Skip the detailed world for basic testing:

```bash
# Launch in empty world (minimal environment)
ros2 launch turtlebot3_gazebo turtlebot3_empty_world.launch.py
```

**Benefits**:
- ~50% less CPU usage
- Faster startup
- Good for testing robot kinematics, sensors, basic navigation

**Trade-offs**:
- No obstacles (can't practice obstacle avoidance)
- No landmarks (limited SLAM testing)

**When to use**:
- Initial robot testing
- Developing custom nodes
- Algorithm prototyping before full simulation

---

### Option 4: Limit Docker Resources

**Impact**: Prevents Docker from consuming all system resources

Restrict Docker resource usage in `docker-compose.yml`:

```yaml
services:
  nav2_dev:
    # ... existing configuration ...
    deploy:
      resources:
        limits:
          cpus: '4.0'      # Limit to 4 CPU cores
          memory: 4G        # Limit to 4GB RAM
        reservations:
          memory: 2G        # Reserve at least 2GB
```

Then restart:
```bash
docker-compose down
docker-compose up -d
```

**Benefits**:
- Prevents Docker from starving other applications
- Ensures system remains responsive
- Forces optimization within constraints

**When to use**:
- Shared development machines
- Systems running multiple applications
- Preventing Docker from causing system freezes

---

### Option 5: Reduce Gazebo Physics Rate

**Impact**: Saves ~40% CPU with minimal simulation accuracy loss

Reduce physics update rate from 1000 Hz to 100 Hz:

```bash
# Inside container
docker exec -it nav2_workspace bash

# Create Gazebo configuration
mkdir -p ~/.gazebo
echo "[physics]
max_step_size = 0.01
real_time_update_rate = 100" > ~/.gazebo/gui.ini
```

**Benefits**:
- Default: 1000 Hz physics → Reduced: 100 Hz
- Still accurate for navigation (Nav2 runs at 10-20 Hz)
- ~40% less CPU usage

**Trade-offs**:
- Slightly less realistic physics
- Not suitable for high-speed maneuvers or precise manipulation

**When to use**:
- Navigation learning (not affected by lower physics rate)
- Low-spec systems
- Background data collection

---

## Recommended Combinations

### For 4-6 GB RAM Systems

**Minimal Configuration** (most aggressive):
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_empty_world.launch.py gui:=false

# Visualize in RViz2 (separate terminal)
rviz2
```

**Resource savings**:
- CPU usage: ~150% → ~30-40%
- RAM usage: ~2.5 GB → ~1.2 GB
- Startup time: ~45s → ~8s

---

### For 6-8 GB RAM Systems

**Balanced Configuration**:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false

# Visualize in RViz2
rviz2
```

**Resource savings**:
- CPU usage: ~150% → ~50-60%
- RAM usage: ~2.5 GB → ~1.5 GB
- Maintains complex world for obstacle avoidance learning

---

### For 8+ GB RAM Systems (Optimization Still Useful)

**Development Configuration**:
```bash
# Use full Waffle model but headless for efficiency
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false

# Use RViz2 for visualization
rviz2
```

**Benefits**:
- Keep all sensors (LiDAR + camera)
- Full world complexity
- Save CPU for faster simulation speeds

---

## Performance Monitoring

### Check Resource Usage

**Monitor Docker container**:
```bash
docker stats nav2_workspace
```

Shows real-time:
- CPU percentage
- Memory usage / limit
- Network I/O

**Inside container, check processes**:
```bash
docker exec -it nav2_workspace bash
top  # Press 'q' to quit

# Or use htop for better UI (if installed)
htop
```

**Check Gazebo FPS**:
- Look at Gazebo window bottom-left corner
- Target: >30 FPS for smooth operation
- Acceptable: >15 FPS for navigation learning
- Poor: <10 FPS (apply optimizations above)

---

## Advanced Optimizations

### Use Custom Launch Files

Create optimized launch configurations in `ros2_ws/src/`:

```python
# minimal_sim.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('turtlebot3_gazebo'),
                '/launch/turtlebot3_world.launch.py'
            ]),
            launch_arguments={
                'gui': 'false',
                'verbose': 'false',
            }.items()
        )
    ])
```

Launch with:
```bash
ros2 launch minimal_sim.launch.py
```

---

### Reduce RViz2 Resource Usage

**Optimize RViz2 visualization**:

1. **Reduce LaserScan visualization**:
   - LaserScan display → Size (m): 0.02 (default 0.03)
   - LaserScan display → Decay Time: 0 (don't keep old scans)

2. **Reduce TF update rate**:
   - TF display → Update Interval: 0.1 (instead of 0)

3. **Disable unused displays**:
   - Uncheck displays you're not actively using
   - Camera images are especially resource-intensive

4. **Reduce grid size**:
   - Grid display → Plane Cell Count: 20 (instead of 100)

---

### Use Bag Files for Testing

Record once, replay many times without running Gazebo:

```bash
# Record simulation data (with Gazebo running)
ros2 bag record -a -o my_test_run

# Later: Replay without Gazebo
ros2 bag play my_test_run

# Test navigation with recorded data
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

**Benefits**:
- No Gazebo overhead during development
- Reproducible test scenarios
- Fast iteration on navigation parameters

---

## Benchmarking Your Configuration

Test different configurations to find the best balance:

```bash
# Benchmark script
#!/bin/bash

echo "Testing configuration: $1"

# Start simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py $1 &
SIM_PID=$!

# Wait for startup
sleep 30

# Measure for 60 seconds
echo "Measuring for 60 seconds..."
docker stats --no-stream --format "table {{.CPUPerc}}\t{{.MemUsage}}" nav2_workspace > stats_$1.txt
sleep 60

# Stop simulation
kill $SIM_PID

echo "Results saved to stats_$1.txt"
```

Usage:
```bash
chmod +x benchmark.sh
./benchmark.sh "gui:=true"
./benchmark.sh "gui:=false"
```

---

## Troubleshooting Performance Issues

### Simulation is very slow (< 10 FPS)

**Immediate fixes**:
1. Enable headless mode: `gui:=false`
2. Switch to Burger model: `export TURTLEBOT3_MODEL=burger`
3. Use empty world: `turtlebot3_empty_world.launch.py`

**System-level fixes**:
```bash
# Close other applications
# Check for background processes
ps aux | grep -E "chrome|firefox|code"

# Increase Docker memory limit (if available)
# Edit docker-compose.yml → deploy.resources.limits.memory: 6G
```

---

### Docker container keeps crashing

**Likely cause**: Out of memory

**Solution**:
1. Check available memory: `free -h`
2. Enable swap if not present (Linux):
   ```bash
   sudo fallocate -l 4G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```
3. Use minimal configuration (headless + burger + empty)

---

### System becomes unresponsive

**Likely cause**: Docker consuming all resources

**Solution**: Apply Docker resource limits (Option 4 above)

**Emergency recovery** (if system freezes):
- SSH from another machine (if possible)
- Force kill Docker: `sudo systemctl restart docker`
- Reboot if necessary

---

## Related Documentation

- [Installation Guide](installation.md) - Setup instructions
- [Troubleshooting Guide](troubleshooting.md) - Common issues
- [ROS2 Essentials](ros2_essentials.md) - Command reference

---

## Summary

**Quick wins** (apply immediately):
- ✅ Use headless Gazebo: `gui:=false` (saves 60% CPU)
- ✅ Use Burger model: `export TURTLEBOT3_MODEL=burger` (saves 30% CPU)
- ✅ Monitor resources: `docker stats nav2_workspace`

**For low-spec systems** (< 6 GB RAM):
- ✅ Combine headless + burger + empty world
- ✅ Reduce physics rate to 100 Hz
- ✅ Limit Docker resources in docker-compose.yml

**Remember**: These optimizations maintain full navigation learning capabilities while making the system accessible to more users!
