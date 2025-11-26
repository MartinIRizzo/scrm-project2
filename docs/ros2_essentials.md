# ROS2 Essentials

Quick reference guide for ROS2 commands and Nav2 workflows.

---

## Getting Help

ROS2 has excellent built-in help:

```bash
# Get help for any ROS2 command
ros2 --help
ros2 <command> --help

# Examples
ros2 topic --help
ros2 node --help
ros2 launch --help
```

---

## Essential ROS2 Commands

### Topics (Pub/Sub Messaging)

Topics are the primary communication mechanism in ROS2.

```bash
# List all active topics
ros2 topic list

# Show topic type
ros2 topic type /scan
# Output: sensor_msgs/msg/LaserScan

# Show topic info (publishers, subscribers)
ros2 topic info /cmd_vel

# Echo topic messages (print to terminal)
ros2 topic echo /scan
ros2 topic echo /odom

# Check topic publish rate
ros2 topic hz /scan
# Should show ~5-30 Hz for LiDAR

# Publish to topic (manual control example)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

---

### Nodes (Running Programs)

Nodes are individual processes in the ROS graph.

```bash
# List all running nodes
ros2 node list

# Show node info (topics, services, actions)
ros2 node info /turtlebot3_node

# Kill a node
ros2 lifecycle set /node_name shutdown
```

---

### Services (Request/Response)

Services provide synchronous request/response communication.

```bash
# List all services
ros2 service list

# Show service type
ros2 service type /spawn_entity

# Call a service
ros2 service call /reset_simulation std_srvs/srv/Empty
```

---

### Parameters (Runtime Configuration)

Parameters configure node behavior at runtime.

```bash
# List parameters for a node
ros2 param list /turtlebot3_node

# Get parameter value
ros2 param get /turtlebot3_node use_sim_time

# Set parameter value
ros2 param set /turtlebot3_node use_sim_time true

# Dump all parameters to file
ros2 param dump /turtlebot3_node > turtlebot3_params.yaml

# Load parameters from file
ros2 param load /turtlebot3_node turtlebot3_params.yaml
```

---

### Actions (Long-Running Tasks)

Actions provide asynchronous goal-based communication (used by Nav2).

```bash
# List all actions
ros2 action list

# Show action type
ros2 action type /navigate_to_pose

# Send action goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5}}}}"

# Show action info
ros2 action info /navigate_to_pose
```

---

### Packages

Packages organize ROS2 code.

```bash
# List all installed packages
ros2 pkg list

# Search for packages
ros2 pkg list | grep nav2
ros2 pkg list | grep turtlebot3

# Show package path
ros2 pkg prefix turtlebot3_gazebo

# List package executables
ros2 pkg executables turtlebot3_gazebo
```

---

### Launch Files (Multi-Node Startup)

Launch files start multiple nodes with configurations.

```bash
# Launch a launch file
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Launch with arguments
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false

# Show launch file arguments
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py --show-args
```

---

### TF (Transform) System

TF manages coordinate frame transformations.

```bash
# List all transforms
ros2 run tf2_ros tf2_echo base_link odom

# View transform tree
ros2 run tf2_tools view_frames
# Generates frames.pdf

# Check if transform exists
ros2 run tf2_ros tf2_echo map base_link
```

---

## Nav2 Workflows

### Complete SLAM and Navigation Workflow

#### Step 1: Launch Simulation

```bash
# Terminal 1: Launch TurtleBot3 in Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

#### Step 2: Run SLAM (Mapping)

```bash
# Terminal 2: Launch SLAM Cartographer
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# Or use SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

#### Step 3: Teleoperate to Build Map

```bash
# Terminal 3: Keyboard control
ros2 run turtlebot3_teleop teleop_keyboard

# Drive around to map the environment
# Use: w (forward), a (left), d (right), x (backward), s (stop)
```

#### Step 4: Save Map

```bash
# Terminal 4: Save the map
ros2 run nav2_map_server map_saver_cli -f /shared/maps/my_map

# This creates:
# - my_map.pgm (image)
# - my_map.yaml (metadata)
```

#### Step 5: Launch Navigation

```bash
# Stop SLAM (Ctrl+C in Terminal 2)

# Terminal 2: Launch Nav2 with saved map
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=True \
    map:=/shared/maps/my_map.yaml
```

#### Step 6: Set Initial Pose (Localization)

```bash
# In RViz2:
# 1. Click "2D Pose Estimate" button
# 2. Click and drag on map where robot is located
# 3. Watch particles converge (AMCL localization)
```

#### Step 7: Send Navigation Goals

```bash
# In RViz2:
# Click "Nav2 Goal" button
# Click destination on map

# Or use command line:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

### Quick Navigation (Pre-Mapped)

If you already have a map:

```bash
# Terminal 1: Launch simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch Nav2
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=True \
    map:=/shared/maps/turtlebot3_world.yaml

# Terminal 3: Launch RViz2 (optional if not running)
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Then set initial pose and send navigation goals in RViz2.

---

### Headless Simulation (No Gazebo GUI)

For resource-constrained systems:

```bash
# Terminal 1: Headless Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false

# Terminal 2: Nav2
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=True \
    map:=/shared/maps/my_map.yaml

# Terminal 3: RViz2 for visualization
rviz2
```

---

## Useful Diagnostics

### Check System Status

```bash
# Overall system check
ros2 doctor

# Check DDS discovery
ros2 multicast receive
# (In another terminal) ros2 multicast send

# Check topics are publishing
ros2 topic list -v
ros2 topic hz /scan
ros2 topic hz /odom
```

---

### Monitor Navigation Status

```bash
# Check if Nav2 is running
ros2 node list | grep nav2

# Monitor navigation action
ros2 action list | grep navigate

# Check costmaps
ros2 topic echo /local_costmap/costmap
ros2 topic echo /global_costmap/costmap

# Check planner status
ros2 topic echo /plan

# Check current pose
ros2 topic echo /amcl_pose
```

---

### Debug Localization

```bash
# Check AMCL particle cloud
ros2 topic echo /particle_cloud

# Check transform tree
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link

# Verify sensor data
ros2 topic hz /scan        # Should be ~5-30 Hz
ros2 topic echo /scan --once
```

---

## Recording and Playback

### Record Simulation Data

```bash
# Record all topics
ros2 bag record -a -o my_simulation

# Record specific topics
ros2 bag record /scan /odom /cmd_vel -o my_nav_test

# Record with compression
ros2 bag record -a --compression-mode file --compression-format zstd
```

---

### Playback Recorded Data

```bash
# Play bag file
ros2 bag play my_simulation

# Play at different speed
ros2 bag play my_simulation --rate 0.5  # Half speed
ros2 bag play my_simulation --rate 2.0  # Double speed

# Play with loop
ros2 bag play my_simulation --loop

# Check bag info
ros2 bag info my_simulation
```

---

## Custom ROS2 Package Development

### Create New Package

```bash
cd /ros2_ws/src

# Python package
ros2 pkg create --build-type ament_python my_nav_pkg

# C++ package
ros2 pkg create --build-type ament_cmake my_nav_cpp_pkg

# With dependencies
ros2 pkg create --build-type ament_python my_nav_pkg \
    --dependencies rclpy nav2_msgs geometry_msgs
```

---

### Build and Run

```bash
cd /ros2_ws

# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_nav_pkg

# Build with symlink-install (for Python, faster iteration)
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Run node
ros2 run my_nav_pkg my_node
```

---

## Common Patterns

### Launch Files for Complex Scenarios

Save as `my_simulation.launch.py` in `ros2_ws/src/`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('turtlebot3_gazebo'),
                '/launch/turtlebot3_world.launch.py'
            ]),
            launch_arguments={'gui': 'false'}.items()
        ),
        # Launch Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('nav2_bringup'),
                '/launch/navigation_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': 'True',
                'map': '/shared/maps/my_map.yaml'
            }.items()
        ),
    ])
```

Launch with:
```bash
ros2 launch my_simulation.launch.py
```

---

## Official Resources

For comprehensive ROS2 tutorials:

- **ROS2 CLI Tutorial**: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html
- **Nav2 Documentation**: https://docs.nav2.org/
- **Nav2 Tutorials**: https://docs.nav2.org/tutorials/index.html
- **ROS2 Launch Files**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html

---

## Quick Reference Card

```bash
# Start simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# SLAM
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# Save map
ros2 run nav2_map_server map_saver_cli -f /shared/maps/my_map

# Navigation
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True map:=/shared/maps/my_map.yaml

# Teleoperation
ros2 run turtlebot3_teleop teleop_keyboard

# Check topics
ros2 topic list

# Check nodes
ros2 node list

# Visualize
rviz2
```

---

## Related Documentation

- [Installation Guide](installation.md) - Setup instructions
- [Troubleshooting Guide](troubleshooting.md) - Common issues
- [Resource Optimization](resource_optimization.md) - Performance tuning
- [Exercises](exercises/) - Hands-on learning materials
