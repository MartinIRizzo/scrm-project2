#!/bin/bash
# Test script for multi-world launch functionality
# Tests that all worlds can be selected and launch file validates correctly

set -e

echo "================================================"
echo "Multi-World Launch Test Suite"
echo "================================================"
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Source ROS2
source /opt/ros/humble/setup.bash
cd /ros2_ws

echo -e "${BLUE}Test 1: Verify worlds.yaml config exists${NC}"
if [ -f "/shared/configs/worlds.yaml" ]; then
    echo -e "${GREEN}✓ Config file exists${NC}"
else
    echo -e "${RED}✗ Config file not found${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Test 2: Verify world files exist${NC}"
worlds=("landmark" "turtlebot3_world" "turtlebot3_house" "empty_world")

for world in "${worlds[@]}"; do
    echo -n "  Checking $world... "
    # Use Python to resolve path
    path=$(python3 << EOPYTHON
import os, yaml
from ament_index_python.packages import get_package_share_directory
with open('/shared/configs/worlds.yaml') as f:
    config = yaml.safe_load(f)
wc = config['worlds']['$world']
if wc['type'] == 'absolute':
    print(wc['path'])
else:
    print(os.path.join(get_package_share_directory(wc['package']), wc['path']))
EOPYTHON
)

    if [ -f "$path" ]; then
        echo -e "${GREEN}✓${NC} ($path)"
    else
        echo -e "${RED}✗${NC} (not found: $path)"
        exit 1
    fi
done

echo ""
echo -e "${BLUE}Test 3: Verify launch file accepts world argument${NC}"
output=$(ros2 launch src/simulation.launch.py --show-args 2>&1)
if echo "$output" | grep -q "world"; then
    echo -e "${GREEN}✓ World argument found${NC}"
else
    echo -e "${RED}✗ World argument not found${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Test 4: Verify default world is landmark${NC}"
if echo "$output" | grep -A 1 "'world':" | grep -q "landmark"; then
    echo -e "${GREEN}✓ Default world is landmark${NC}"
else
    echo -e "${RED}✗ Default world is not landmark${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Test 5: Verify available worlds listed in help${NC}"
if echo "$output" | grep -q "landmark, turtlebot3_world, turtlebot3_house, empty_world"; then
    echo -e "${GREEN}✓ All worlds listed in help text${NC}"
else
    echo -e "${RED}✗ Not all worlds listed${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Test 6: Python syntax validation${NC}"
if python3 -m py_compile src/simulation.launch.py 2>/dev/null; then
    echo -e "${GREEN}✓ Launch file syntax valid${NC}"
else
    echo -e "${RED}✗ Launch file has syntax errors${NC}"
    exit 1
fi

echo ""
echo "================================================"
echo -e "${GREEN}All tests passed!${NC}"
echo "================================================"
echo ""
echo "Manual verification steps:"
echo "1. ros2 launch src/simulation.launch.py"
echo "2. ros2 launch src/simulation.launch.py world:=turtlebot3_world"
echo "3. ros2 launch src/simulation.launch.py world:=turtlebot3_house"
echo "4. ros2 launch src/simulation.launch.py world:=empty_world"
echo ""
echo "In each case, verify:"
echo "  - Gazebo launches without errors"
echo "  - Correct world loads visually"
echo "  - Robot spawns successfully"
echo "  - Topics publish: ros2 topic list | grep -E '(scan|camera)'"
