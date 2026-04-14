cmake_minimum_required(VERSION 3.8)
project(immkf_predictor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -O3)
endif()

# Find dependencies FIRST
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Generate ROS2 messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryPoint.msg"
  "msg/PredictedTrajectory.msg"
  "msg/IMMKFState.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
  ADD_LINTER_TESTS
)

# Core motion models library
add_library(immkf_core_lib SHARED
  src/motion_models.cpp
  src/immkf_filter.cpp
  src/data_association.cpp
)
target_include_directories(immkf_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(immkf_core_lib Eigen3::Eigen)
ament_target_dependencies(immkf_core_lib rclcpp)

# Tracker library
add_library(immkf_tracker_lib SHARED src/immkf_tracker.cpp)
target_include_directories(immkf_tracker_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(immkf_tracker_lib immkf_core_lib Eigen3::Eigen)
ament_target_dependencies(immkf_tracker_lib rclcpp)

# Main node library
add_library(immkf_node_lib SHARED src/immkf_node.cpp)
target_include_directories(immkf_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(immkf_node_lib
  immkf_tracker_lib
  immkf_core_lib
  Eigen3::Eigen
)
ament_target_dependencies(immkf_node_lib
  rclcpp
  rclcpp_components
  geometry_msgs
  visualization_msgs
  yolo_msgs
  tf2
  tf2_geometry_msgs
)

# Link the generated interface
rosidl_target_interfaces(immkf_node_lib
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

# Test executable for synthetic trajectories
add_executable(test_synthetic_trajectory test/test_motion_models.cpp)
target_include_directories(test_synthetic_trajectory PRIVATE include)
target_link_libraries(test_synthetic_trajectory
  immkf_core_lib
  Eigen3::Eigen
)

# Install
install(TARGETS
  immkf_core_lib
  immkf_tracker_lib
  immkf_node_lib
  test_synthetic_trajectory
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(
  Eigen3
  rosidl_default_runtime
)

ament_package()
