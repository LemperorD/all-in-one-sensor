cmake_minimum_required(VERSION 3.8)
project(sync_board_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Protobuf REQUIRED)
find_package(fmt REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

include_directories(
    ../include
    include
    ../third_party/cxxopts
    ../third_party/mavlink
    ../proto_generated/cpp
)

# Sources
set(SOURCES
    src/sync_board.cpp
    src/serial_port.cpp
    src/mavlink_handler.cpp
    src/mavlink_sender.cpp
    src/sync_core.cpp
    ../proto_generated/cpp/sync.pb.cc
)

add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES} fmt::fmt pthread)