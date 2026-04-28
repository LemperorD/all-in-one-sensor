cmake_minimum_required(VERSION 3.8)
project(simulator)

find_package(ament_cmake_auto REQUIRED)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Find package
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Gazebo Fortress
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

message(STATUS "Compiling against Ignition Fortress")

# Include
include_directories(include)

# Create gz_cam lib
add_library(gz_cam_node SHARED
  src/gz_cam_node.cpp
)
set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  rmoss_interfaces
  image_transport
  ros_gz_bridge
)
ament_target_dependencies(gz_cam_node ${dependencies})
target_link_libraries(gz_cam_node
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
)
add_executable(gz_cam src/gz_cam_main.cpp)
target_link_libraries(gz_cam gz_cam_node)
rclcpp_components_register_nodes(gz_cam_node "gz_cam::GzCamNode")

# Create gz_gimbal lib
add_library(gz_gimbal_node SHARED
  src/gz_gimbal_actuator.cpp
  src/gz_gimbal_encoder.cpp
  src/pid.cpp
  src/gimbal_controller.cpp
)
set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  example_interfaces
  tf2_ros
)
ament_target_dependencies(gz_gimbal_node ${dependencies})
target_link_libraries(gz_gimbal_node
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
)
add_executable(gz_gimbal src/gz_gimbal_main.cpp)
target_link_libraries(gz_gimbal gz_gimbal_node)
rclcpp_components_register_nodes(gz_gimbal_node "gz_gimbal::GzGimbalNode")

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gimbal.msg"
  "msg/GimbalCmd.msg"
  "srv/GetCameraInfo.srv"
DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  sensor_msgs
)

ament_auto_package(
  INSTALL_TO_SHARE
  resource
  config
  launch
)
