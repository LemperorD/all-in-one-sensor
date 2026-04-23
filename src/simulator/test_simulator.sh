#!/bin/bash
# Simulator test script

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=== Gimbal Simulator Test Suite ===${NC}"
echo ""

# Timeout for each test (seconds)
TEST_TIMEOUT=30

# Test counter
TESTS_PASSED=0
TESTS_FAILED=0

# Helper function for testing
run_test() {
    local test_name="$1"
    local test_cmd="$2"
    local expected_result="$3"  # "success" or "output_contains"

    echo -e "${YELLOW}Testing: $test_name${NC}"

    # Run the command with timeout
    if timeout $TEST_TIMEOUT bash -c "$test_cmd" 2>&1 | tee /tmp/test_output.log; then
        if [[ "$expected_result" == "success" ]]; then
            echo -e "${GREEN}✓ PASSED${NC}"
            ((TESTS_PASSED++))
        else
            echo -e "${RED}✗ FAILED${NC}"
            ((TESTS_FAILED++))
        fi
    else
        echo -e "${RED}✗ FAILED (timeout or error)${NC}"
        ((TESTS_FAILED++))
    fi
    echo ""
}

# Test 1: Check ROS environment
echo -e "${BLUE}--- Environment Tests ---${NC}"
run_test "ROS 2 environment" "test -n \$ROS_DISTRO && echo 'ROS_DISTRO is set'" "success"

# Test 2: Check simulator package
echo -e "${BLUE}--- Package Tests ---${NC}"
run_test "Simulator package installed" "ros2 pkg prefix simulator" "success"

# Test 3: Check executables
echo -e "${BLUE}--- Executable Tests ---${NC}"
run_test "gimbal_controller_node exists" "which gimbal_controller_node || ros2 run simulator gimbal_controller_node --help" "success"
run_test "target_tracker_node exists" "which target_tracker_node || ros2 run simulator target_tracker_node --help" "success"
run_test "gazebo_bridge_node exists" "which gazebo_bridge_node || ros2 run simulator gazebo_bridge_node --help" "success"

# Test 4: Check configuration files
echo -e "${BLUE}--- Configuration Tests ---${NC}"
SIMULATOR_SHARE=$(ros2 pkg prefix simulator)/share/simulator
run_test "Camera info config exists" "test -f $SIMULATOR_SHARE/config/camera_info.yaml" "success"
run_test "Lidar config exists" "test -f $SIMULATOR_SHARE/config/lidar_config.yaml" "success"
run_test "Gimbal controllers config exists" "test -f $SIMULATOR_SHARE/config/gimbal_controllers.yaml" "success"

# Test 5: Check URDF files
echo -e "${BLUE}--- URDF/Model Tests ---${NC}"
run_test "Gimbal platform URDF exists" "test -f $SIMULATOR_SHARE/urdf/gimbal_platform.urdf" "success"
run_test "Gimbal plugins config exists" "test -f $SIMULATOR_SHARE/urdf/gimbal_plugins.gazebo" "success"

# Test 6: Check model files
echo -e "${BLUE}--- Model Definition Tests ---${NC}"
run_test "Gimbal platform model exists" "test -f $SIMULATOR_SHARE/models/gimbal_platform/model.sdf" "success"
run_test "QR code target model exists" "test -f $SIMULATOR_SHARE/models/qr_code_target/model.sdf" "success"

# Test 7: Check launch files
echo -e "${BLUE}--- Launch File Tests ---${NC}"
run_test "Gazebo sim launch file" "test -f $SIMULATOR_SHARE/launch/gazebo_sim.launch.py" "success"
run_test "Full system launch file" "test -f $SIMULATOR_SHARE/launch/full_system.launch.py" "success"

# Test 8: Check world files
echo -e "${BLUE}--- World File Tests ---${NC}"
run_test "Gazebo Sim SDF file" "test -f $SIMULATOR_SHARE/worlds/gimbal_sim.sdf" "success"
run_test "Gazebo Sim GUI config" "test -f $SIMULATOR_SHARE/ign/gui.config" "success"

# Test 9: Check ROS topic infrastructure
echo -e "${BLUE}--- ROS Topic Tests ---${NC}"
run_test "Can list ROS packages" "ros2 pkg list | grep simulator" "success"
run_test "Can find dependencies" "ros2 pkg depends simulator | head -5" "success"

# Summary
echo ""
echo -e "${BLUE}=== Test Summary ===${NC}"
echo -e "${GREEN}Passed: $TESTS_PASSED${NC}"
echo -e "${RED}Failed: $TESTS_FAILED${NC}"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}All tests passed!${NC}"
    exit 0
else
    echo -e "${RED}Some tests failed. Please check the output above.${NC}"
    exit 1
fi
