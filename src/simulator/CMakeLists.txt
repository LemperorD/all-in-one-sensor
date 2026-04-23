cmake_minimum_required(VERSION 3.8)
project(simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_sim.dsv.in")

# Gimbal Controller Node
add_executable(gimbal_controller_node src/gimbal_controller_node.cpp)
ament_target_dependencies(gimbal_controller_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Target Tracker Node - simulates moving target trajectory
add_executable(target_tracker_node src/target_tracker_node.cpp)
ament_target_dependencies(target_tracker_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  yolo_msgs
)

# Gazebo bridge node - publishes sensor data from Gazebo
add_executable(gazebo_bridge_node src/gazebo_bridge_node.cpp)
ament_target_dependencies(gazebo_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
  camera_info_manager
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  OpenCV
)

install(TARGETS
  gimbal_controller_node
  target_tracker_node
  gazebo_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install model files
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

# Install world files
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

# Install URDF files
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

# Install ignition files
install(DIRECTORY ign DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpylint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
