# Exercise 3: Navigation Basics

**Session**: 3
**Duration**: 60-90 minutes
**Prerequisites**: Docker environment running (see [Docker Setup Guide](../docker_setup.md))
**Learning Objectives**:
- Launch ROS2 simulation with Nav2 stack
- Initialize robot localization using AMCL
- Send navigation goals via RViz and command line
- Interpret RViz displays for navigation monitoring
- Troubleshoot common navigation issues

---

## Overview

This exercise guides you through launching a complete autonomous navigation system using ROS2 Nav2. You will start a simulated TurtleBot3 robot in Gazebo, initialize its position using AMCL (Adaptive Monte Carlo Localization), and command the robot to navigate to goal positions.

By the end of this exercise, you will understand the multi-component architecture of Nav2 and be able to operate a fully autonomous mobile robot.

---

## Part 1: Launch Simulation Environment (15 min)

### Step 1: Start the Simulation

Open a new terminal and launch the Gazebo simulation with the TurtleBot3 robot:

```bash
docker exec -it nav2_workspace bash
ros2 launch /ros2_ws/src/simulation.launch.py headless:=true
```

**Launch Options Explained:**
- `headless:=true` - Runs Gazebo without GUI (recommended, saves resources)
- `headless:=false` - Shows Gazebo GUI for visualization
- `rviz:=false` - Disables RViz auto-launch if you want to start it manually later

**Expected Output:**
```
[INFO] [launch]: All log files can be found below /root/.ros/log/...
[INFO] [gzserver-1]: process started with pid [...]
[INFO] [robot_state_publisher-2]: process started with pid [...]
[INFO] [spawn_entity.py-3]: process started with pid [...]
```

**What's Happening:**
- Gazebo server starts with the `landmark_world.world`
- Robot state publisher broadcasts TF transforms
- TurtleBot3 Waffle spawns at position (-2.0, 2.0, 0.0)

**Verification:**
```bash
# In a new terminal
docker exec -it nav2_workspace bash
ros2 topic list | grep -E "(odom|scan|cmd_vel)"
```

You should see:
- `/odom` - Odometry data from wheel encoders
- `/scan` - Laser scan data from LiDAR
- `/cmd_vel` - Velocity commands to the robot

---

## Part 2: Launch Nav2 Stack (15 min)

### Step 2: Start Nav2 Navigation System

Open a **second terminal** and launch the Nav2 navigation stack:

```bash
docker exec -it nav2_workspace bash
ros2 launch /ros2_ws/src/nav2.launch.py
```

**Expected Output:**
```
[INFO] [lifecycle_manager_navigation]: Creating
[INFO] [lifecycle_manager_localization]: Creating
[INFO] [map_server]: Creating
[INFO] [amcl]: Creating
[INFO] [controller_server]: Creating
[INFO] [planner_server]: Creating
[INFO] [behavior_server]: Creating
[INFO] [bt_navigator]: Creating
...
[INFO] [lifecycle_manager_navigation]: Activating
[INFO] [lifecycle_manager_localization]: Activating
```

**What's Happening:**
- **Map Server**: Loads `/shared/maps/landmark_map.yaml` for localization
- **AMCL**: Starts particle filter localization (initially uninitialized)
- **Controller Server**: Local trajectory generation (DWB controller)
- **Planner Server**: Global path planning (Theta* planner)
- **Behavior Server**: Recovery behaviors (spin, backup, wait)
- **BT Navigator**: Coordinates navigation via behavior trees

**Wait for:** The `[lifecycle_manager_navigation]: Activating` message before proceeding.

---

## Part 3: Initialize Robot Localization (15 min)

### Step 3: Initial Pose (Automatic)

AMCL requires an initial pose estimate to start localization. The Nav2 launch file automatically publishes the initial pose 5 seconds after startup.

**What happens automatically:**
- Position matches the robot's spawn location from `simulation_params.yaml`
- Publishes 3 times to ensure reliable delivery
- Sets initial uncertainty via covariance matrix

**Verification:**

Check the **Nav2 terminal (Terminal 2)** for these messages:
```
[amcl]: initialPoseReceived
[amcl]: Setting pose (3 covariance terms are zero)
```

If you see these messages, AMCL has successfully initialized!

---

## Part 4: RViz Navigation Interface (20 min)

### Step 4: Understand RViz Displays

RViz should have auto-launched with the Nav2 stack. If not visible, check your X11 forwarding setup.

**Default RViz Displays:**

| Display | Description | Color | What to Look For |
|---------|-------------|-------|------------------|
| **Map** | Static map from map server | Grayscale | Black = obstacles, white = free space |
| **Particle Cloud** | AMCL localization particles | Green arrows | Should cluster around robot after initialization |
| **Global Path** | Planned path to goal | Red line | Full path from current pose to goal |
| **Local Path** | Controller trajectory | Blue line | Short-term path for obstacle avoidance |
| **Global Costmap** | Long-term planning costs | Red/yellow/blue | Red = high cost (obstacles) |
| **Local Costmap** | Short-term control costs | Red/yellow/blue | Updates in real-time with sensor data |
| **LaserScan** | LiDAR sensor data | Red points | Shows detected obstacles |
| **TF** | Coordinate frames | Colored axes | Shows robot frame relationships |

**Troubleshooting: Map Not Displaying**

If the map appears blank in RViz:
1. Click on the **Map** display in the left panel
2. Expand **Topic** section
3. Change **Durability Policy** from `Volatile` to `Transient Local`
4. The map should now appear

**Why?** The map publisher uses `TRANSIENT_LOCAL` QoS (late-joining subscribers receive last message), but RViz defaults to `VOLATILE` (no historical messages).

---

## Part 5: Send Navigation Goals (20 min)

### Step 5A: GUI-Based Navigation (RViz)

1. In RViz, click the **"Nav2 Goal"** button in the top toolbar (target icon with arrow)
2. Click on a free space in the map (white area)
3. Drag to set the desired orientation (arrow direction)
4. Release the mouse button

**Expected Behavior:**
- Red global path appears from robot to goal
- Blue local path updates as robot moves
- Robot starts moving toward the goal
- Green particle cloud follows the robot

**Good First Goal Coordinates:**
- From spawn (-2.0, 2.0) → Try (1.0, -2.0) - diagonal path southeast

### Step 5B: Command-Line Navigation

Alternatively, send goals via the command line for precise control:

```bash
# In Terminal 3 (or new terminal)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 1.0, y: -2.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}" --feedback
```

**With Feedback:** The `--feedback` flag shows real-time navigation progress:
```
Feedback:
  current_pose:
    pose:
      position:
        x: -1.85
        y: 1.92
  distance_remaining: 1.23
  ...
```

**Goal Result:**
```
Result:
  result: Goal reached!
  status: SUCCEEDED
```

---

## Part 6: Monitoring and Debugging (15 min)

### Step 6: Monitor Robot State

Open additional terminals to inspect the navigation system:

**Watch Robot Pose:**
```bash
ros2 topic echo /amcl_pose
```

Output shows estimated position and orientation from AMCL:
```yaml
pose:
  pose:
    position:
      x: -1.95
      y: 2.01
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.01
      w: 0.999
```

**List All Active Topics:**
```bash
ros2 topic list
```

Key topics to recognize:
- `/amcl_pose` - Localization output
- `/cmd_vel` - Velocity commands to robot
- `/scan` - LiDAR data
- `/map` - Static map
- `/global_costmap/costmap` - Planning costs
- `/local_costmap/costmap` - Control costs
- `/plan` - Global path
- `/local_plan` - Local trajectory

**Check Active Nodes:**
```bash
ros2 node list
```

Expected Nav2 nodes:
- `/amcl` - Localization
- `/bt_navigator` - Behavior tree coordinator
- `/controller_server` - Trajectory control
- `/planner_server` - Path planning
- `/behavior_server` - Recovery behaviors
- `/map_server` - Map provider

---

## Troubleshooting Guide

### Problem: Map Not Displaying in RViz

**Symptoms:** RViz shows black grid instead of map

**Solution:**
1. Select **Map** display in left panel
2. Expand **Topic** → **Durability Policy**
3. Change to **Transient Local**

**Root Cause:** QoS mismatch between publisher (transient local) and subscriber (volatile default)

---

### Problem: AMCL Warnings "cannot publish pose"

**Symptoms:** Nav2 terminal shows repeated warnings about AMCL not publishing

**Solution:**
1. Run the initialization command from Part 3 again
2. Publish 3 times: `--rate 1 --times 3`
3. Check for `[amcl]: initialPoseReceived` in Nav2 terminal
4. If still failing, restart Nav2 launch (Ctrl+C in Terminal 2, relaunch)

**Root Cause:** AMCL missed the initial pose message due to timing

---

### Problem: Initialization Not Working

**Symptoms:** No `initialPoseReceived` message after publishing

**Solution:**
1. Wait 2-3 seconds after Nav2 starts before initializing
2. Use `--times 3` to publish multiple times
3. Verify topic exists: `ros2 topic info /initialpose`
4. Check AMCL is running: `ros2 node info /amcl`

**Root Cause:** AMCL subscriber not ready or message missed due to timing

---

### Problem: Navigation Goals Abort Immediately

**Symptoms:** Goal accepted but aborts with "Goal failed" or "No valid path"

**Possible Causes and Solutions:**

1. **Goal in Obstacle:**
   - Check RViz costmaps (red = high cost)
   - Choose goal in white/blue area (free space)

2. **Robot Not Localized:**
   - Verify particle cloud converged around robot (not scattered)
   - Re-initialize pose if particles are spread out

3. **Goal Too Close to Obstacle:**
   - Inflation layer adds safety margin around obstacles
   - Try goal further from walls/objects

4. **Costmap Configuration:**
   - Check `/shared/configs/nav2_params.yaml`
   - Ensure `inflation_radius` is reasonable (default: 0.5m)

---

## Exercise Tasks

Complete the following tasks to demonstrate mastery:

### Task 1: Basic Navigation (Required)
1. Launch simulation in headless mode
2. Start Nav2 stack
3. Initialize robot pose at (-2.0, 2.0, 0.0)
4. Send navigation goal to (1.0, -2.0, 0.0) via RViz
5. Verify robot reaches goal successfully

### Task 2: Command-Line Navigation (Required)
1. Use `ros2 action send_goal` to navigate to (1.0, -2.0)
2. Capture the feedback output
3. Verify goal success in terminal output

### Task 3: Monitoring (Required)
1. While robot is navigating, echo `/amcl_pose` in another terminal
2. List all active topics and count Nav2-related topics
3. Identify the controller and planner nodes

---

## Key Concepts Review

After completing this exercise, you should understand:

1. **Multi-Process Architecture:** Nav2 uses multiple processes (simulation, navigation, nodes) working together
2. **Localization Initialization:** AMCL requires initial pose estimate to start particle filter
3. **Goal Specification:** Goals include position (x, y, z) and orientation (quaternion)
4. **Path Planning vs. Control:** Global planner creates full path, local controller handles obstacle avoidance
5. **Costmaps:** Multi-layered representation of environment for navigation decisions

---

## File Locations Reference

**Simulation Launch:**
```
/ros2_ws/src/simulation.launch.py
```

**Nav2 Launch:**
```
/ros2_ws/src/nav2.launch.py
```

**Map File:**
```
/shared/maps/landmark_map.yaml
/shared/maps/landmark_map.pgm
```

**Nav2 Configuration:**
```
/shared/configs/nav2_params.yaml
```

**RViz Configuration:**
```
/shared/configs/default.rviz
```

---

## Next Steps

In the next exercise, you will:
- Learn SLAM (Simultaneous Localization and Mapping)
- Create your own maps using Cartographer
- Understand the difference between mapping and localization
- Save and load custom maps for navigation

**Preparation:**
- Keep the current simulation running if you want to explore further
- Try navigating to different areas of the map
- Experiment with different goal orientations
- Observe how the robot handles narrow passages

---

## Additional Resources

**Nav2 Documentation:**
- Getting Started Guide: https://docs.nav2.org/getting_started/
- Navigation Concepts: https://docs.nav2.org/concepts/

**AMCL Documentation:**
- AMCL Overview: https://docs.nav2.org/configuration/packages/configuring-amcl.html
- Particle Filter Theory: https://en.wikipedia.org/wiki/Particle_filter

**ROS2 Actions:**
- Understanding Actions: https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Actions.html
- Action Client Tutorial: https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Py-Action-Client.html

**ROS2 QoS:**
- QoS Policies: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
- Durability Explained: https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html
