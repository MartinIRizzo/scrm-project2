# Installation Guide

Complete setup guide for the MUSI-nav2 ROS2 Navigation learning environment.

---

## System Requirements

### Standard Configuration
- **Operating System**: Ubuntu 22.04 (native or WSL2)
- **Docker**: Version 20.10 or higher
- **RAM**: 8 GB minimum (16 GB recommended)
- **Disk Space**: 10 GB free space
- **Display**: X11 server for GUI applications

### Low-Spec Configuration (4-6 GB RAM)
Same as above, but use resource-saving options:
- Headless Gazebo + lightweight robot model
- Can run on systems with 4 GB RAM (with reduced features)
- See [Resource Optimization Guide](resource_optimization.md) for details

---

## Platform-Specific Setup

### Linux (Ubuntu 20.04/22.04)

#### 1. Install Docker and Docker Compose

```bash
# Install Docker
sudo apt update
sudo apt install -y docker.io docker-compose

# Add your user to docker group (logout/login required after this)
sudo usermod -aG docker $USER

# Verify installation
docker --version
docker-compose --version
```

**Important**: After adding yourself to the docker group, you must logout and login again for changes to take effect.

#### 2. X11 Server

X11 is already installed on Ubuntu Desktop. No additional setup needed.

---

### WSL2 (Windows)

#### 1. Docker Desktop for Windows (Recommended)

1. Download and install [Docker Desktop](https://www.docker.com/products/docker-desktop)
2. During installation, ensure **"Use WSL 2 based engine"** is enabled
3. After installation:
   - Open Docker Desktop → Settings → Resources → WSL Integration
   - Enable integration with your Ubuntu distribution (e.g., Ubuntu-20.04)
4. Start Docker Desktop (it must be running for Docker commands to work in WSL)

**Verify installation in WSL terminal**:
```bash
docker --version
docker-compose --version

# Check Docker is running
docker ps
```

**Important**: Docker Desktop must be running on Windows before using Docker commands in WSL.

#### 2. X11 Server (for GUI applications)

**Windows 11**: WSLg is built-in (no setup needed)

**Windows 10**: Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) or [X410](https://x410.dev/)
- Launch XLaunch with settings:
  - Multiple windows
  - Start no client
  - Disable access control
- Add to `~/.bashrc`:
  ```bash
  export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}):0
  ```

---

### macOS

#### 1. Docker Desktop for Mac

1. Download and install [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop)
2. Docker Desktop includes Docker Compose
3. Start Docker Desktop from Applications

**Verify installation**:
```bash
docker --version
docker-compose --version
```

#### 2. X11 Server (for GUI applications)

1. Install [XQuartz](https://www.xquartz.org/)
2. Launch XQuartz → Preferences → Security → Enable "Allow connections from network clients"
3. Restart XQuartz
4. In terminal:
   ```bash
   # Allow X11 forwarding
   xhost +localhost

   # Set DISPLAY variable
   export DISPLAY=host.docker.internal:0
   ```

**Note**: On Apple Silicon (M1/M2/M3), Docker will use Rosetta 2 for x86_64 images. Performance may vary.

---

## Build and Setup

### Step 1: Clone the Repository

```bash
git clone https://github.com/ericguerrero/MUSI-nav2.git
cd MUSI-nav2
```

### Step 2: Build the Docker Environment

```bash
./build.sh
```

The build process will:
- Create necessary directories (`ros2_ws/`, `shared/`)
- Build the Docker image (~4 minutes, ~4.2 GB)
- Install ROS2 Humble, Nav2, TurtleBot3, and all dependencies

### Step 3: Start the Environment

```bash
./run.sh
```

This will:
- Enable X11 forwarding for GUI applications
- Start the Docker container in the background
- Mount your workspace for persistent data

### Step 4: Access the Container

```bash
# Open a terminal inside the container
docker exec -it nav2_workspace bash
```

You're now inside the ROS2 environment with all packages ready!

---

## Verification

### Test Docker Installation

```bash
# Inside the container, check ROS2 packages
ros2 pkg list | grep nav2          # Should show 20+ Nav2 packages
ros2 pkg list | grep turtlebot3    # Should show 38+ TurtleBot3 packages
```

### Test GUI Forwarding

```bash
# Inside container
rviz2
```

If RViz2 window appears, GUI forwarding is working correctly.

### Test Simulation

```bash
# Launch TurtleBot3 in Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Wait 30-60 seconds** for Gazebo to fully load. You should see a Gazebo window with a simulated indoor environment and a TurtleBot3 Waffle robot.

**Note**: If you see "not responding" dialog, click "Wait" - this is normal during first launch while shaders compile.

### Verify ROS2 Topics

After launching the simulation, check active topics:

```bash
# In a new terminal (also inside container)
docker exec -it nav2_workspace bash

# List active topics
ros2 topic list
# Should show /scan, /odom, /cmd_vel, /camera/image_raw, etc.

# Check TF frames
ros2 run tf2_tools view_frames
# Generates frames.pdf with transform tree
```

---

## Environment Management

### Start Environment
```bash
./run.sh
```

### Stop Environment
```bash
./stop.sh
```

Workspace data is preserved across restarts.

### Rebuild After Dockerfile Changes
```bash
docker-compose down
./build.sh
./run.sh
```

### Access Multiple Terminals

```bash
# Terminal 1: Launch simulation
docker exec -it nav2_workspace bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Run navigation
docker exec -it nav2_workspace bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

# Terminal 3: Send commands
docker exec -it nav2_workspace bash
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## Common Issues

For troubleshooting Docker, X11, and simulation issues, see [Troubleshooting Guide](troubleshooting.md).

Quick troubleshooting:

**Docker not found**: Install Docker following platform instructions above

**Permission denied on scripts**: Run `chmod +x build.sh run.sh stop.sh`

**GUI doesn't show**: Check X11 setup in [Troubleshooting Guide](troubleshooting.md)

**Container starts then exits**: Check logs with `docker logs nav2_workspace`

---

## Next Steps

Once your environment is running:

1. **Follow Nav2 Official Tutorials**: https://docs.nav2.org/tutorials/index.html
2. **Explore ROS2 Basics**: [ROS2 Essentials Guide](ros2_essentials.md)
3. **Try Our Exercises**: See [docs/exercises/](exercises/) for hands-on learning
4. **Optimize for Low-Spec Systems**: See [Resource Optimization Guide](resource_optimization.md)

---

## Directory Structure Reference

```
MUSI-nav2/
├── Dockerfile              # Docker image definition
├── docker-compose.yml      # Container configuration
├── build.sh                # Build automation script
├── run.sh                  # Startup script
├── stop.sh                 # Shutdown script
│
├── ros2_ws/                # Your ROS2 workspace (persistent)
│   └── src/                # Put your custom ROS2 packages here
│
└── shared/                 # Shared resources (accessible from host and container)
    ├── bags/               # ROS2 bag recordings (for data logging)
    ├── configs/            # Nav2 parameter configurations
    ├── maps/               # SLAM-generated maps (.pgm and .yaml files)
    └── worlds/             # Custom Gazebo world files
```

---

## Additional Resources

- **Nav2 Documentation**: https://docs.nav2.org/
- **ROS2 Humble Documentation**: https://docs.ros.org/en/humble/
- **TurtleBot3 Manual**: https://emanual.robotis.com/docs/en/platform/turtlebot3/
