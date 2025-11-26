# Exercise 5: SLAM Parameter Variations and Algorithm Comparison

**Objective**: Explore how SLAM parameter choices affect map quality, computational cost, and real-time performance through hands-on experimentation.

---

## Background: Why SLAM Parameter Tuning Matters

SLAM (Simultaneous Localization and Mapping) is not a one-size-fits-all solution. Different applications have different requirements:

- **Warehouse robots** need high-precision maps but have powerful computers
- **Delivery drones** need real-time performance with limited CPU
- **Research platforms** need maximum detail for analysis

In this exercise, you'll test three parameter configurations:
- **Balanced**: Good quality with moderate resource usage (default)
- **High Detail**: Maximum quality for precision applications
- **Realtime**: Fast performance for resource-constrained robots

---

## Available Simulation Worlds

**turtlebot3_world**

![TurtleBot3 World Map](../../shared/maps/turtlebot3_world_gazebo.png)

**turtlebot3_house**

![TurtleBot3 House Map](../../shared/maps/turtlebot3_house_gazebo.png)

---

## Part A: Parameter Sensitivity Study with Cartographer

### Overview

You'll map the same environment three times with different Cartographer configurations, then compare the results.

**Key Parameters Being Varied:**

| Parameter | Balanced | High Detail | Realtime |
|-----------|----------|-------------|----------|
| **Resolution** | 0.05m | 0.025m | 0.10m |
| **Max Range** | 3.5m | 12.0m | 6.0m |
| **Voxel Filter** | 0.025 | 0.01 | 0.10 |
| **Motion Filter** | 20cm/10° | 10cm/5° | 50cm/20° |

### Step 1: Map with Balanced Configuration (Baseline)

**Terminal 1: Start Simulation**
```bash
docker exec -it nav2_workspace bash
cd /ros2_ws/src
ros2 launch simulation.launch.py world:=turtlebot3_world rviz:=false
```

Wait for Gazebo to load completely (robot spawned, no errors).

**Terminal 2: Launch Cartographer with Balanced Config**
```bash
docker exec -it nav2_workspace bash
cd /ros2_ws/src
ros2 launch slam_cartographer.launch.py configuration_basename:=cartographer_balanced.lua
```

RViz should open showing the robot and an empty map.

**Terminal 3: Teleoperation**
```bash
docker exec -it nav2_workspace bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Mapping Instructions:**
- Drive for **2-3 minutes**
- Cover the entire visible area
- Drive slowly and smoothly
- Make a **figure-8 or loop** to test loop closure

**Observe in RViz:**
- Map building in real-time
- Robot position updating
- Laser scans matching map features

**Terminal 4: Save the Map**

Once mapping is complete, save the map:
```bash
docker exec -it nav2_workspace bash
ros2 run nav2_map_server map_saver_cli -f /shared/maps/test/cart_balanced
```

Expected output:
```
[INFO] [map_saver]: Saving map from 'map' topic to '/shared/maps/cart_balanced' file
[INFO] [map_saver]: Map saved
```

**Check the files created:**
```bash
ls -lh /shared/maps/test/cart_balanced.*
```

You should see:
- `cart_balanced.pgm` (map image)
- `cart_balanced.yaml` (map metadata)

### Step 2: Map with High Detail Configuration

**Stop Cartographer** (Ctrl+C in Terminal 2)

**Restart robot position in Gazebo:**
- In Gazebo window: Edit → Reset Model Poses
- OR: Restart simulation (Ctrl+C Terminal 1, then relaunch)

**Terminal 2: Launch High Detail Config**
```bash
cd /ros2_ws/src
ros2 launch slam_cartographer.launch.py configuration_basename:=cartographer_high_detail.lua
```

**Terminal 3: Drive the Same Pattern**

Try to replicate your previous driving pattern as closely as possible.

**Check CPU usage**: `htop` in another terminal (look for cartographer_node)

**Save the Map:**
```bash
ros2 run nav2_map_server map_saver_cli -f /shared/maps/cart_high_detail
```

### Step 3: Map with Realtime Configuration

**Repeat the process with realtime config:**

```bash
# Terminal 2:
ros2 launch slam_cartographer.launch.py configuration_basename:=cartographer_realtime.lua

# Terminal 3: Drive same pattern

# Terminal 4: Save map
ros2 run nav2_map_server map_saver_cli -f /shared/maps/cart_realtime
```

---

## Part B: Parameter Sensitivity Study with SLAM Toolbox

### Overview

Now repeat the parameter study with SLAM Toolbox to compare algorithm behavior.

**Key Parameters Being Varied:**

| Parameter | Balanced | High Detail | Realtime |
|-----------|----------|-------------|----------|
| **Resolution** | 0.05m | 0.025m | 0.10m |
| **Max Range** | 3.5m | 12.0m | 6.0m |
| **Min Travel Distance** | 0.2m | 0.1m | 0.5m |
| **Min Travel Heading** | 0.17 rad | 0.087 rad | 0.35 rad |
| **Loop Search Distance** | 3.0m | 5.0m | 3.0m |

### Step 1: Map with Balanced Configuration

**Stop Cartographer and restart simulation**

**Terminal 2: Launch SLAM Toolbox Balanced**
```bash
cd /ros2_ws/src
ros2 launch slam_nav2.launch.py slam_params_file:=/shared/configs/slamtoolbox_balanced.yaml
```

**Terminal 3: Drive the same pattern** (2-3 minutes)

**Save the map:**
```bash
ros2 run nav2_map_server map_saver_cli -f /shared/maps/slamtb_balanced
```

### Step 2: Map with High Detail Configuration

**Restart simulation and robot position**

**Terminal 2: Launch High Detail**
```bash
cd /ros2_ws/src
ros2 launch slam_nav2.launch.py slam_params_file:=/shared/configs/slamtoolbox_high_detail.yaml
```

**Terminal 3: Drive the same pattern**

**Save the map:**
```bash
ros2 run nav2_map_server map_saver_cli -f /shared/maps/slamtb_high_detail
```

### Step 3: Map with Realtime Configuration

**Restart simulation and robot position**

**Terminal 2: Launch Realtime**
```bash
cd /ros2_ws/src
ros2 launch slam_nav2.launch.py slam_params_file:=/shared/configs/slamtoolbox_realtime.yaml
```

**Terminal 3: Drive the same pattern**

**Save the map:**
```bash
ros2 run nav2_map_server map_saver_cli -f /shared/maps/slamtb_realtime
```

---

## Part C: Algorithm Comparison (Cartographer vs SLAM Toolbox)

### Overview

Compare the two SLAM algorithms at the same configuration level (balanced).

**Create a comparison table in your lab notebook:**

| Metric | Cartographer | SLAM Toolbox |
|--------|-------------|--------------|
| **Launch time** | ___ seconds | ___ seconds |
| **CPU usage (avg)** | ___% | ___% |
| **Loop closure speed** | Fast/Slow | Fast/Slow |
| **Map file size** | ___ KB | ___ KB |
| **Visual quality** | Better/Worse/Same | Better/Worse/Same |

**Questions to answer:**
1. Which algorithm detected loop closure faster when you completed the loop?
2. Which algorithm used more CPU resources?
3. Which map looks more consistent/accurate to you?
4. Which would you choose for a robot with limited CPU?

---

## Part D: Quantitative Analysis

### Compare Map Files

**Check file sizes:**
```bash
ls -lh /shared/maps/*.pgm
```

Record in your table:

| Config | Algorithm | File Size | Resolution | Notes |
|--------|-----------|-----------|------------|-------|
| cart_balanced | Cartographer | ___ KB | 0.05m | |
| cart_high_detail | Cartographer | ___ KB | 0.025m | |
| cart_realtime | Cartographer | ___ KB | 0.10m | |
| slamtb_balanced | SLAM Toolbox | ___ KB | 0.05m | |
| slamtb_high_detail | SLAM Toolbox | ___ KB | 0.025m | |
| slamtb_realtime | SLAM Toolbox | ___ KB | 0.10m | |

### Visual Comparison

**Open maps side-by-side:**

You can view the PGM files with any image viewer:
```bash
# If on Linux with GUI:
eog /shared/maps/cart_balanced.pgm /shared/maps/cart_high_detail.pgm /shared/maps/cart_realtime.pgm
```

Or copy them to your host machine and open in any image editor.

---

## Deliverables

### Required Submission

Submit a report (PDF or Markdown) containing:

#### Part A: Cartographer Configuration Comparison

**Configuration Comparison Table:**
| Configuration | Resolution | File Size | CPU Usage | Visual Quality | Best Use Case |
|---------------|-----------|-----------|-----------|----------------|---------------|
| Balanced | | | | | |
| High Detail | | | | | |
| Realtime | | | | | |

#### Part B: SLAM Toolbox Configuration Comparison

**Configuration Comparison Table:**
| Configuration | Resolution | File Size | CPU Usage | Visual Quality | Best Use Case |
|---------------|-----------|-----------|-----------|----------------|---------------|
| Balanced | | | | | |
| High Detail | | | | | |
| Realtime | | | | | |

#### Part C: Algorithm Comparison (Cartographer vs SLAM Toolbox)

**Quantitative Metrics:**

Use the provided metrics script to analyze your maps:
```bash
python3 /shared/scripts/slam_metrics.py /shared/maps/cart_balanced.yaml
python3 /shared/scripts/slam_metrics.py /shared/maps/slamtb_balanced.yaml
```

The script calculates:
- **Map entropy** - Information density (higher = more detail captured)
- **Coverage** - Percentage of map explored (not unknown)
- **Obstacle density** - Ratio of obstacles to free space
- **Map efficiency** - Information per storage unit (cells/KB)
- **Overall quality score** - Weighted average (0-100 scale)

**Note:** The metrics script (`/shared/scripts/slam_metrics.py`) is an initial implementation provided as a starting point. You are encouraged to review, modify, and improve it. Consider adding your own metrics or adjusting the quality score calculation based on what you think matters most for SLAM evaluation.

**Additional Metrics to Consider:**
- File size (KB)
- CPU usage during mapping (%)
- Loop closure detection time
- Visual quality assessment

**Written comparison:**
- Which algorithm produced better quantitative metrics?
- Which was more CPU efficient?
- When would you choose each algorithm?

#### Part D: Analysis Questions

**Resolution Trade-offs:**
- Why doesn't higher resolution always mean better mapping?
- What are the practical limitations of 0.025m resolution?
- When would 0.10m resolution be acceptable?

**Parameter Tuning Strategy:**
- You have a robot with limited CPU (old laptop). Which parameters would you adjust first?
- Your warehouse robot needs precision obstacle detection. Which config would you choose and why?
- How would you tune parameters for a fast-moving delivery robot?

**SLAM Algorithm Selection:**
- Based on your tests, when would you use Cartographer?
- When would SLAM Toolbox be better?
- What factors matter most: CPU, accuracy, or loop closure speed?

#### Screenshots

Include labeled screenshots showing:
- RViz during mapping (map building)
- Representative maps from each configuration
- CPU usage comparison (optional: htop screenshot)

#### Bonus: Metrics Script Improvements

**Optional Deliverable:** If you modified or improved the metrics script, include:
- Your modified script file (`slam_metrics_improved.py`)
- Documentation of what you changed and why
- Comparison of original vs. improved metrics on your maps

This demonstrates deeper understanding of SLAM evaluation criteria.

---

## Tips for Success

### Driving Patterns

**For best results:**
- Drive **slowly and smoothly** (don't jerk the controls)
- Make **complete loops** to test loop closure
- Cover the **entire area** systematically
- Avoid **rapid spinning** (confuses scan matching)
- Drive **close to walls** initially, then open areas

### Troubleshooting

**Problem: Map looks noisy/inconsistent**
- You drove too fast → Slow down
- Loop didn't close → Drive a complete loop
- Wheel slip occurred → Avoid sudden turns

**Problem: Cartographer won't launch**
- Check config file path is correct
- Verify you're in `/ros2_ws/src` directory
- Check Docker container is running: `docker ps`

**Problem: Map won't save**
- Ensure `/map` topic is publishing: `ros2 topic echo /map --once`
- Wait for map to be built before saving
- Check file permissions in `/shared/maps/`

**Problem: High Detail config is very slow**
- This is expected! It's processing 4x more data
- Reduce driving speed to match processing speed
- Consider using `realtime` config instead on slow hardware

---

## Resources

### Configuration Files Location
- Cartographer: `/shared/configs/cartographer_*.lua`
- SLAM Toolbox: `/shared/configs/slamtoolbox_*.yaml`

### Launch Files Location
- SLAM launchers: `/ros2_ws/src/slam_*.launch.py`
- Simulation: `/ros2_ws/src/simulation.launch.py`

### Documentation
- [Cartographer Documentation](https://google-cartographer-ros.readthedocs.io/)
- [SLAM Toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 SLAM Integration](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)

---
