FROM osrf/ros:humble-desktop-full

# Metadata for documentation
LABEL maintainer="Nav2 Student Tutorial"
LABEL description="Complete ROS2 Nav2 development environment for students"
LABEL version="1.0"

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install all necessary packages in single layer for efficiency
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-slam-toolbox \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    git \
    vim \
    nano \
    tmux \
    wget \
    eog \
    curl \
    htop \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for Nav2 examples and SLAM metrics
RUN pip3 install --no-cache-dir transforms3d numpy pillow pyyaml scipy

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Environment setup for TurtleBot3
ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models
ENV ROS_DOMAIN_ID=0

CMD ["/bin/bash"]
