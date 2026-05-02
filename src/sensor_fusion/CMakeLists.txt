cmake_minimum_required(VERSION 3.8)
project(sensor_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ByteTrack library
add_library(bytetrack SHARED
  src/bytetrack.cpp
)

target_include_directories(bytetrack
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(bytetrack
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

target_compile_features(bytetrack PUBLIC cxx_std_17)

# AB3DMOT library (3D Multi-Object Tracking)
add_library(ab3dmot SHARED
  src/AB3DMOT.cpp
)

target_include_directories(ab3dmot
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(ab3dmot
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

target_compile_features(ab3dmot PUBLIC cxx_std_17)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(
  TARGETS bytetrack ab3dmot
  DESTINATION lib
)

# Fusion node (component) and fusion utils
add_library(fusion_node SHARED
  src/fusion_node.cpp
  src/fusion_utils.cpp
)

target_include_directories(fusion_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fusion_node rclcpp rclcpp_components)

target_link_libraries(fusion_node
  bytetrack
  ab3dmot
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

target_compile_features(fusion_node PUBLIC cxx_std_17)

install(TARGETS fusion_node DESTINATION lib)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
