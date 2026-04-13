<div align="center"> 

# Muti Sensor Sync Board SDK
</div>

Muti Sensor Sync Board 多传感器时间同步板的SDK，实现以下功能：

- 提供网络时间源
- 相机单次触发、帧率控制
- 触发时间戳获取
- 传感器数据读取
- 时间源配置

## Getting started

```bash
git clone https://github.com/emNavi/sync_board_sdk.git
```

## Build

### linux
#### 安装依赖
```bash
sudo apt install -y libprotobuf-dev protobuf-compiler libspdlog-dev libboost-all-dev
```
#### Protobuf 代码生成：

```bash
cd sync_board_sdk/
bash generate_proto.sh
```
生成`sync.pb.cc`、`sync.pb.h`

#### 编译
```bash
cd sync_board_sdk/
mkdir build && cd build
cmake ..
make
```

## Use
#### udev
固定设备名称 '/dev/ttyEmnvSyncBoard'
```
sudo cp ./udev/99_emnavi.rules /etc/udev/rules.d/99_emnavi.rules
sudo udevadm control --reload
sudo udevadm trigger
```

### 方式一：运行示例程序
```bash
cd build/bin/examples
./camera_control
```

### 方式二：使用SDK动态库
将以下内容复制到你的项目中：
```css
build/libsync_board_core.so
sync_board_core/include/
third_party/mavlink/
```
项目结构可以如下：
```css
your_project/
├── src/
│   └── ...
├── include/
│   └── ...
├── sync_board_core/
│   └── include/
├── lib/
│   └── libsync_board_core.so
├── third_party/
│   └── mavlink/
└── CMakeLists.txt
```
CMakeLists.txt 示例：
```cmake
cmake_minimum_required(VERSION 3.10)
project(your_project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# 包含 SDK 头文件
include_directories(${CMAKE_SOURCE_DIR}/sync_board_core/include)
# 包含 MAVLINK 头文件
include_directories(${CMAKE_SOURCE_DIR}/third_party/mavlink)
# 链接库路径
link_directories(${CMAKE_SOURCE_DIR}/lib)

add_executable(your_project src/main.cpp)

# 链接 SDK 动态库
target_link_libraries(your_project sync_board_core pthread)
```
参考examples/ 中的代码进行使用




### 方式三：使用 sync client core 

见 `sync_client/readme.md`