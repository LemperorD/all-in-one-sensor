cmake_minimum_required(VERSION 3.8)
project(sync_board_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(fmt REQUIRED)

# Include directories
include_directories(include)
include_directories(sync_board_core/include)
include_directories(third_party/cxxopts)
include_directories(proto_generated/cpp)
include_directories(third_party/mavlink)

# Add subdirectories for SDK
add_subdirectory(sync_board_core)
# add_subdirectory(sync_client)

# Create executable
add_executable(sync_board_node src/sync_board_node.cpp)
ament_target_dependencies(sync_board_node rclcpp sensor_msgs std_msgs std_srvs)
target_link_libraries(sync_board_node sync_board_core fmt::fmt)

install(TARGETS
  sync_board_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when a copyright is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
