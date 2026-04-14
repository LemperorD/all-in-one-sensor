# Gazebo Gimbal Tracking Simulation Platform

一个基于Gazebo的二轴云台多传感器跟踪仿真平台，集成了激光雷达、相机以及MPC视角规划模块。

## 功能特性

- **二轴云台仿真**：Pan（偏航）和Tilt（俯仰）关节
- **多传感器**：模拟激光雷达（16线点云）和RGB相机
- **目标跟踪**：支持多种目标轨迹（圆形、8字形、螺旋上升等）
- **轨迹预测**：生成预测轨迹供MPC规划器使用
- **ROS 2集成**：完整的ROS 2通信接口

## 系统架构

```
Gazebo Simulator
    ├── Gimbal Platform (Pan/Tilt Joints)
    ├── Camera Sensor (RGB, 960x720)
    ├── Lidar Sensor (16-line, 360° scanning)
    └── QR Code Target (Movable)

ROS 2 Nodes
    ├── gazebo_bridge_node       (传感器数据发布)
    ├── gimbal_controller_node   (云台控制)
    ├── target_tracker_node      (目标轨迹模拟)
    ├── mpc_view_planner_node    (MPC视角规划)
    └── lidar_detector/sensor_fusion_node (传感器融合)
```

## 文件结构

```
simulator/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── gimbal_platform.urdf      # 云台硬件描述
│   └── gimbal_plugins.gazebo     # Gazebo传感器插件
├── models/
│   ├── gimbal_platform/
│   │   ├── model.sdf
│   │   └── model.config
│   └── qr_code_target/
│       ├── model.sdf
│       └── model.config
├── config/
│   ├── gimbal_controllers.yaml   # 云台控制器配置
│   ├── camera_info.yaml          # 相机内参
│   └── lidar_config.yaml         # 激光雷达配置
├── worlds/
│   └── gimbal_sim.world          # Gazebo世界文件
├── src/
│   ├── gimbal_controller_node.cpp    # 云台控制
│   ├── target_tracker_node.cpp       # 目标轨迹
│   └── gazebo_bridge_node.cpp        # 传感器桥接
└── launch/
    ├── gazebo_sim.launch.py      # 启动Gazebo
    └── full_system.launch.py     # 启动完整系统
```

## 快速开始

### 1. 编译
```bash
cd ~/all-in-one-sensor
colcon build --packages-select simulator
```

### 2. 启动仿真平台

#### 仅启动Gazebo
```bash
ros2 launch simulator gazebo_sim.launch.py
```

#### 启动完整系统（Gazebo + 所有节点）
```bash
# 圆形炤迹
ros2 launch simulator full_system.launch.py trajectory_type:=circular trajectory_radius:=5.0 trajectory_height:=2.0

# 8字形轨迹
ros2 launch simulator full_system.launch.py trajectory_type:=figure_8

# 螺旋上升轨迹
ros2 launch simulator full_system.launch.py trajectory_type:=spiral_up
```

### 3. 在另一个终端启动MPC规划器和检测模块

```bash
# 启动MPC视角规划器
ros2 run mpc_gimbal_planner mpc_view_planner_node \
    --ros-args -p mpc_horizon:=10 \
    -p planning_period:=0.1 \
    -p w_tracking:=1.0 \
    -p w_smoothness:=0.5 \
    -p w_control:=0.2

# 启动传感器融合检测
ros2 run lidar_detector sensor_fusion_node \
    --ros-args -p dbscan_eps:=0.1 \
    -p dbscan_min_pts:=5
```

### 4. 使用RViz可视化

```bash
rviz2 -d ~/all-in-one-sensor/src/simulator/config/rviz_config.rviz
```

## ROS 2 话题

### 发布（Publishers）

| 话题名 | 消息类型 | 发布节点 | 描述 |
|--------|---------|---------|------|
| `/predicted_trajectory` | yolo_msgs/DetectionArray | target_tracker_node | 预测的目标轨迹 |
| `/sim_target_detection` | yolo_msgs/DetectionArray | target_tracker_node | 当前目标检测 |
| `/gimbal_state` | std_msgs/Float32MultiArray | gimbal_controller_node | 云台状态 (pan, tilt) |
| `/camera_info` | sensor_msgs/CameraInfo | gazebo_bridge_node | 相机内参 |
| `/camera/image_rect` | sensor_msgs/Image | Gazebo Plugin | 相机图像 |
| `/lidar_points` | sensor_msgs/PointCloud2 | Gazebo Plugin | 激光雷达点云 |

### 订阅（Subscribers）

| 话题名 | 消息类型 | 订阅节点 | 描述 |
|--------|---------|---------|------|
| `/gimbal_command` | geometry_msgs/TwistStamped | gimbal_controller_node | MPC规划的云台命令 |
| `/joint_states` | sensor_msgs/JointState | gimbal_controller_node | Gazebo关节状态 |
| `/gazebo/model_states` | gazebo_msgs/ModelStates | target_tracker_node | Gazebo模型状态 |

## 配置参数

### gimbal_controller_node

```yaml
max_pan_rate: 2.0          # 最大偏航角速度 (rad/s)
max_tilt_rate: 2.0         # 最大俯仰角速度 (rad/s)
control_period: 0.01       # 控制周期 (s)
```

### target_tracker_node

```yaml
trajectory_type: "circular"     # 轨迹类型: circular, figure_8, spiral_up
trajectory_radius: 5.0          # 轨迹半径 (m)
trajectory_height: 2.0          # 目标高度 (m)
trajectory_period: 20.0         # 完整轨迹周期 (s)
publish_rate: 10.0              # 发布速率 (Hz)
```

### 相机参数

内参矩阵（camera_info.yaml）：
```
fx = 1470.0, fy = 1470.0
cx = 480.0, cy = 360.0
分辨率: 960 x 720
```

### 激光雷达参数

- 水平采样点：360
- 竖直采样线：16
- 水平FOV：360°
- 竖直FOV：±15°
- 最大范围：25 m
- 更新率：10 Hz

## 集成流程

### 1. 数据流
```
Gazebo Sensors
    ↓
gazebo_bridge_node (发布ROS消息)
    ↓
lidar_detector::sensor_fusion_node (传感器融合)
    ↓
yolo_msgs/DetectionArray (融合检测)
    ↓
mpc_gimbal_planner::MPCViewPlannerNode (视角规划)
    ↓
geometry_msgs/TwistStamped (云台命令)
    ↓
gimbal_controller_node (执行控制)
    ↓
Gazebo (更新云台关节)
```

### 2. 闭环跟踪
- 目标轨迹由 `target_tracker_node` 生成并发布为预测轨迹
- MPC规划器接收预测轨迹，计算最优云台命令
- 云台控制器执行命令，更新云台姿态
- Gazebo更新传感器视角，获得新的传感器读数

## 扩展建议

### 近期（Phase 1）
- [ ] 实现PID视觉伺服控制作为对照组
- [ ] 添加更多目标轨迹选项
- [ ] 集成YOLO检测模块进行真实检测
- [ ] 生成轨迹预测和规划结果的数据日志

### 中期（Phase 2）
- [ ] 替换QR码为无人机模型
- [ ] 添加更复杂的场景（多目标、遮挡等）
- [ ] 实现检测精度评估模块
- [ ] 添加RViz可视化配置

### 长期（Phase 3）
- [ ] 仿真与实物对比实验
- [ ] 参数优化和敏感性分析
- [ ] 实时性能剖析
- [ ] 导出仿真数据集用于深度学习

## 故障排除

### Gazebo无法启动
```bash
# 检查环保GAZEBO_MODEL_PATH
echo $GAZEBO_MODEL_PATH

# 确保RViz不与Gazebo冲突
killall gzclient gzserver
```

### 传感器无数据
```bash
# 检查话题是否发布
ros2 topic list | grep camera
ros2 topic list | grep lidar

# 查看话题数据
ros2 topic echo /camera/image_rect --once
ros2 topic echo /lidar_points --once
```

### 云台不动作
```bash
# 检查关节状态
ros2 topic echo /joint_states

# 检查命令是否发布
ros2 topic echo /gimbal_command
```

## 参考资源

- [ROS 2 官方文档](https://docs.ros.org/)
- [Gazebo 模拟器](http://gazebosim.org/)
- [URDF 规范](http://wiki.ros.org/urdf/XML)

## 许可证

Apache License 2.0

## 联系方式

维护者：ld
邮箱：ld2382619813@163.com
