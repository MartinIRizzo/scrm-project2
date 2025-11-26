#!/bin/bash

echo "🚀 Starting SCRM container..."
echo ""

# Check if image exists
if [[ "$(docker images -q nav2_student:humble 2> /dev/null)" == "" ]]; then
    echo "❌ Docker image 'nav2_student:humble' not found."
    echo "Please run ./build.sh first to build the image."
    exit 1
fi

# Enable X11 forwarding
echo "🖥️  Enabling X11 forwarding for GUI applications..."
xhost +local:docker > /dev/null 2>&1

# Start container
echo "🐳 Starting Docker container..."
docker compose up -d

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ SCRM development environment started!"
    echo ""
    echo "Access the container:"
    echo "  docker exec -it scrm-project2 bash"
    echo ""
    echo "Stop environment:"
    echo "  ./stop.sh"
else
    echo "❌ Failed to start container. Check Docker daemon status."
    exit 1
fi
