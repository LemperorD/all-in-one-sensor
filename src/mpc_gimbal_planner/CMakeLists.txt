cmake_minimum_required(VERSION 3.8)
project(mpc_gimbal_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_auto_find_build_dependencies()

find_package(ACADO REQUIRED)

# MPC View Planner library
add_library(mpc_view_planner SHARED src/mpc_view_planner.cpp)
target_include_directories(mpc_view_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mpc_view_planner
  Eigen3::Eigen
)
ament_target_dependencies(mpc_view_planner)

# MPC View Planner ROS2 Node
add_library(mpc_view_planner_node SHARED src/mpc_view_planner_node.cpp)
target_include_directories(mpc_view_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mpc_view_planner_node
  mpc_view_planner
  Eigen3::Eigen
)
ament_target_dependencies(mpc_view_planner_node
  rclcpp
  rclcpp_components
  geometry_msgs
  yolo_msgs
  std_msgs
)
rclcpp_components_register_node(mpc_view_planner_node
  PLUGIN "mpc_gimbal_planner::MPCViewPlannerNode"
  EXECUTABLE mpc_view_planner_executable
)

# Install
install(TARGETS mpc_view_planner DESTINATION lib)
install(TARGETS mpc_view_planner_node DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalState.msg"
)

ament_export_dependencies(
  rosidl_default_runtime
  Eigen3
)

ament_package()
