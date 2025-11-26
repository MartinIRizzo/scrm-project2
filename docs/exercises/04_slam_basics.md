# Exercise 4: Autonomous Waypoint Navigation

**Duration**: 30-45 minutes
**Prerequisites**: Completed Exercise 3 (Navigation Basics)
**Learning Objectives**:
- Understand sequential waypoint navigation
- Program autonomous patrol routes
- Modify waypoint sequences for custom behaviors

---

## Overview

This exercise introduces autonomous waypoint following. Instead of manually sending individual navigation goals, the robot will follow a sequence of waypoints automatically - useful for patrol routes, delivery tasks, and inspection tours.

---

## Setup (Quick Review from Exercise 3)

### Terminal 1: Start Simulation
```bash
docker exec -it nav2_workspace bash
ros2 launch /ros2_ws/src/simulation.launch.py headless:=true
```

### Terminal 2: Start Nav2 Stack
```bash
docker exec -it nav2_workspace bash
ros2 launch /ros2_ws/src/nav2.launch.py
```

Wait for: `[lifecycle_manager_navigation]: Activating` message

### Terminal 3: Initialize Robot Pose
```bash
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {
      position: {x: -2.0, y: 2.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}' --rate 1 --times 3
```

Verify in Terminal 2: `[amcl]: initialPoseReceived`

---

## Part 1: Manual Teleoperation with Nav2

Before running autonomous navigation, practice manual control with Nav2 active.

### Terminal 4: Launch Teleoperation
```bash
docker exec -it nav2_workspace bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Keyboard Controls:**
```
   w/x - increase/decrease linear velocity
   a/d - increase/decrease angular velocity
   s   - stop
   space - emergency stop
```

**Duration:** 5-10 minutes of practice

**When done:** Press `Ctrl+C` to stop teleoperation

---

## Part 2: Autonomous Waypoint Navigation

### Terminal 5: Launch Waypoint Navigation
```bash
docker exec -it nav2_workspace bash
ros2 launch /ros2_ws/src/waypoint_navigation.launch.py
```

**What happens:**
- Robot autonomously navigates through 5 predefined waypoints
- Progress shown in terminal: "Navigating to waypoint X/5"
- Returns to starting position
- Takes approximately 2-3 minutes to complete

**Success message:**
```
[INFO] [waypoint_follower]: SUCCESS! Completed all 5 waypoints!
```

---

## Part 3: SLAM Toolbox with Teleoperation

Now learn to build maps from scratch using SLAM - no pre-built map required!

### Setup (Simulation Already Running)

Assuming simulation from Part 2 is still active. If not, restart:
```bash
docker exec -it nav2_workspace bash
ros2 launch /ros2_ws/src/simulation.launch.py headless:=true
```

### Terminal 2: Launch SLAM Toolbox

```bash
docker exec -it nav2_workspace bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/shared/configs/slam_toolbox_params.yaml \
  use_sim_time:=true
```

**Expected Output:**
```
[slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[slam_toolbox]: Registering sensor: [Custom Described Lidar]
```

### Terminal 3: Initialize Robot Pose

```bash
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {
      position: {x: -2.0, y: 2.0, z: 0.0},
      orientation: {w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
  }
}'
```

### Terminal 4: Manual Exploration with Teleop

```bash
docker exec -it nav2_workspace bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Task:** Drive around for 10-15 minutes to build a complete map

**Exploration Tips:**
- Drive slowly for better scan matching
- Follow walls systematically
- Cover all areas and corners
- Return to start for loop closure

**When done:** Press `Ctrl+C`

### Terminal 5: Save the Map

```bash
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: {data: '/shared/maps/my_slam_map'}"
```

**Expected:** `result: True`

**What you built:**
- Real-time map from scratch
- No pre-built map needed
- SLAM simultaneously localized and mapped

**For detailed instructions:** See `Instructions_Detailed.md` - Session 4, Part 3

---

## Part 4: SLAM-Based Navigation

Use SLAM for autonomous navigation - robot navigates while building/updating map in real-time.

### Setup

**Stop SLAM from Part 3:** Press `Ctrl+C` in Terminal 2 (SLAM Toolbox)

### Terminal 2: Launch SLAM + Nav2 Stack

```bash
docker exec -it nav2_workspace bash
ros2 launch /ros2_ws/src/slam_nav2.launch.py
```

**Expected Output:**
```
[slam_toolbox]: Registering sensor
[controller_server]: Activating
[planner_server]: Activating
[bt_navigator]: Activating
[lifecycle_manager_navigation]: Managed nodes are active
```

**What's different:** SLAM Toolbox replaces AMCL for localization.

### Terminal 3: Initialize Robot Pose

```bash
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {
      position: {x: -2.0, y: 2.0, z: 0.0},
      orientation: {w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
  }
}'
```

### Terminal 4: Optional Pre-Exploration (Recommended)

Build initial map coverage before autonomous navigation:

```bash
docker exec -it nav2_workspace bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Drive around for 5 minutes, then press `Ctrl+C`

### Terminal 5: Test Single Navigation Goal

```bash
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 1.0, y: 1.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}"
```

**Observe:** Robot navigates while building/updating map dynamically

### Terminal 6: Waypoint Following with SLAM

```bash
docker exec -it nav2_workspace bash
python3 /ros2_ws/src/waypoint_follower.py
```

**Expected:**
```
[waypoint_follower]: Navigating to waypoint 1/5
[waypoint_follower]: Navigating to waypoint 2/5
...
[waypoint_follower]: SUCCESS! Completed all 5 waypoints!
```

**Key Difference from Part 2:**
- No pre-built map required
- Map builds during navigation
- May be slower initially (limited map)
- Better for unknown environments

**For detailed instructions:** See `Instructions_Detailed.md` - Session 4, Part 4

---

## Summary

You've learned to:
- ✅ Practice manual control with teleoperation
- ✅ Navigate autonomously through multiple waypoints
- ✅ Understand the progression from manual to autonomous control
- ✅ Ready for advanced SLAM-based navigation
