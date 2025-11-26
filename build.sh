#!/bin/bash

echo "🚀 Building SCRM project 2 Docker Image..."
echo ""

# Create directory structure
mkdir -p ros2_ws/src
mkdir -p shared/maps
mkdir -p shared/configs
mkdir -p shared/bags
mkdir -p shared/student_work

echo "📁 Directory structure created"
echo ""

# Build with error checking
echo "🐳 Building Docker image (this may take 5-10 minutes)..."
docker build -t nav2_student:humble .

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Build complete! Image: nav2_student:humble"
    echo ""
    echo "Next steps:"
    echo "  1. Run: ./run.sh"
    echo "  2. Access container: docker exec -it scrm-project2 bash"
    echo "  3. Inside container: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
else
    echo ""
    echo "❌ Build failed. Please check Docker daemon and error messages above."
    exit 1
fi
