#!/bin/bash
# Simulation Platform Quick Start Script

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Gimbal Simulator Quick Start ===${NC}"
echo ""

# Check ROS 2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS 2 environment not sourced${NC}"
    echo "Please run: source /opt/ros/\$ROS_VERSION/setup.bash"
    exit 1
fi

echo -e "${GREEN}ROS Distribution: $ROS_DISTRO${NC}"

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname $(dirname $(dirname $SCRIPT_DIR)))"

echo -e "${GREEN}Workspace: $WORKSPACE_DIR${NC}"

# Build if needed
echo ""
echo -e "${YELLOW}Building simulator package...${NC}"
cd "$WORKSPACE_DIR"
colcon build --packages-select simulator --symlink-install

if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}Build successful!${NC}"
echo ""

# Source the setup
echo -e "${YELLOW}Sourcing setup...${NC}"
source "$WORKSPACE_DIR/install/setup.bash"

echo ""
echo -e "${GREEN}=== Simulator Ready ===${NC}"
echo ""
echo "Usage examples:"
echo ""
echo "1. Launch full system (Gazebo + all nodes):"
echo -e "   ${YELLOW}ros2 launch simulator full_system.launch.py${NC}"
echo ""
echo "2. Launch with custom trajectory:"
echo -e "   ${YELLOW}ros2 launch simulator full_system.launch.py trajectory_type:=spiral_up${NC}"
echo ""
echo "3. Launch only Gazebo:"
echo -e "   ${YELLOW}ros2 launch simulator gazebo_sim.launch.py${NC}"
echo ""
echo "4. View simulation in RViz:"
echo -e "   ${YELLOW}rviz2 -d $SCRIPT_DIR/config/rviz_config.rviz${NC}"
echo ""
echo "5. Monitor topics:"
echo -e "   ${YELLOW}ros2 topic list${NC}"
echo ""
echo "6. Check MPC planner connection:"
echo -e "   ${YELLOW}ros2 topic echo /predicted_trajectory${NC}"
echo ""
