cmake_minimum_required(VERSION 3.8)
project(lidar_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

# DBSCAN library
add_library(dbscan src/dbscan.cpp)
target_include_directories(dbscan PUBLIC include)

# Sensor fusion node
add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)
ament_target_dependencies(sensor_fusion_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  yolo_msgs
  pcl_ros
  pcl_conversions
  message_filters
)
target_link_libraries(sensor_fusion_node dbscan ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(sensor_fusion_node PRIVATE include)

install(TARGETS sensor_fusion_node DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpylint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
