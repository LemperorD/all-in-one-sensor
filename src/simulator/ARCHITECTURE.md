# Simulator 仿真平台系统架构

## 概览

```
┌──────────────────────────────────────────────────────────────────┐
│                     Gazebo 物理仿真引擎                            │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │   Gimbal Platform   │  │  QR Target   │  │  Ground      │     │
│  │  (2 DOF arm)        │  │  (moving)    │  │  Plane       │     │
│  │  - Pan Joint        │  │              │  │  (static)    │     │
│  │  - Tilt Joint       │  │              │  │              │     │
│  │  - Camera (RGB)     │  │              │  │              │     │
│  │  - Lidar (16-line)  │  │              │  │              │     │
│  └─────────────────────┘  └──────────────┘  └──────────────┘     │
│                                                                  │
│                 Sensor Publishing (30Hz Camera)                  │
│                                (10Hz Lidar)                      │
└────────┬─────────────────────────────────────────────────┬───────┘
         │                                                 │
         ▼                                                 ▼
    /camera/image_rect                            /lidar_points
    /camera_info                                  (PointCloud2)
         │                                                 │
         └────────────────────┬────────────────────────────┘
                              │
                    ┌─────────▼──────────┐
                    │  gazebo_bridge_node│  (ROS ↔ Gazebo)
                    │  - TF publishing   │
                    │  - Camera info     │
                    └────────────────────┘
                              │
              ┌───────────────┼───────────────┐
              │               │               │
    /camera/image_rect  /camera_info   /lidar_points
              │               │               │
    ┌─────────▼────────┐      │         ┌─────▼──────────┐
    │   YOLO Module    │◄─────┘         │  Lidar Module  │
    │ (Optional: Real  │                │  (Clustering)  │
    │  Detection)      │                │  with DBSCAN   │
    └────────┬─────────┘                └────────┬───────┘
             │ Detections          Clusters     │
             └────────────┬─────────────────────┘
                          │
                  ┌───────▼─────────┐
                  │ sensor_fusion   │  (Lidar Detector)
                  │ node            │  Fused detections
                  └────────┬────────┘
                           │
                   ┌───────▼─────────────────────┐
                   │  Predicted Trajectory       │
                   │  (from target_tracker_node) │
                   └───────┬─────────────────────┘
                           │
                   ┌───────▼──────────────────┐
                   │ MPC View Planner Node    │  (MPC Gimbal Planner)
                   │ - Trajectory prediction  │
                   │ - Optimal gimbal commands│
                   └───────┬──────────────────┘
                           │
                  /gimbal_command (angular vel)
                           │
           ┌───────────────▼──────────────┐
           │ gimbal_controller_node       │
           │ - Converts vel to position   │
           │ - Joint saturation          │
           │ - State feedback            │
           └───────────────┬──────────────┘
                           │
                    /gimbal_state (pan, tilt)
                           │
                  ┌────────▼────────┐
                  │ Gazebo Joint    │
                  │ Controller      │
                  └────────┬────────┘
                           │
                    Update Cloud Gimbal
                           │
                (Loop back to Sensor Publishing)
```

## 节点拓扑

```
╔════════════════════════════════════════════════════════════════╗
║                     ROS 2 节点拓扑                             ║
╠════════════════════════════════════════════════════════════════╣

gimbal_controller_node                 target_tracker_node
        /gimbal_state ──────────►  /predicted_trajectory
             │                              │
             │                              │
    /gimbal_command                        │
             │                              │
             └──────────────┬───────────────┘
                            │
                   mpc_view_planner_node
                            │
                  /predicted_trajectory
                            │
                   gazebo_bridge_node
                            │
              /camera_info, /camera/image_rect
              /lidar_points, Transform
                            │
            ┌───────────────┴───────────────┐
            │                               │
   sensor_fusion_node              YOLO Detection
   (lidar_detector)                (optional)
            │
        /fused_detections

data_logger_node (optional)
  - Logs all topics to JSON
  - Performance metrics

╚════════════════════════════════════════════════════════════════╝
```

## 数据流动

### 1. 传感器 → 检测

```
Target Position (Gazebo)
        ↓
Camera Image ────────► YOLO Detection
        │              bbox (2D)
        │
Lidar Point Cloud ────► DBSCAN Clustering
        │              clusters
        │
        └────────────► Sensor Fusion
                       (match detections + clusters)
                       │
                    Fused 3D Detection
```

### 2. 检测 → 规划

```
Fused Detection
        ↓
Target Tracker (predict future positions)
        ↓ (10Hz)
Predicted Trajectory
        ↓
MPC Planner (10 step horizon, 10 constraints)
        ↓
Optimal Gimbal Command
  (pan_rate, tilt_rate)
```

### 3. 规划 → 控制 → 执行

```
Gimbal Command (TwistStamped)
        ↓
Gimbal Controller (integrate rate → position)
        ↓
Target Angles
        ↓
Gazebo Joint Controller
        ↓
Update Pan/Tilt Joints
        ↓
Update Sensor Frame
        ↓
(Loop back to 1)
```

## 文件结构详解

```
simulator/
├── CMakeLists.txt ........................ 构建配置
├── package.xml ........................... 包依赖声明
│
├── src/ .................................. 源代码
│   ├── gimbal_controller_node.cpp ........ 云台控制（C++）
│   ├── target_tracker_node.cpp ........... 目标轨迹（C++）
│   ├── gazebo_bridge_node.cpp ............ 传感器桥接（C++）
│   └── data_logger_node.py ............... 数据记录（Python）
│
├── urdf/ .................................. 硬件模型
│   ├── gimbal_platform.urdf .............. 云台URDF描述
│   └── gimbal_plugins.gazebo ............. Gazebo传感器插件
│
├── models/ ................................ Gazebo模型
│   ├── gimbal_platform/
│   │   ├── model.sdf ..................... Platform SDF
│   │   └── model.config .................. 模型配置
│   └── qr_code_target/
│       ├── model.sdf ..................... QR码 SDF
│       └── model.config .................. 模型配置
│
├── config/ ................................ 配置文件
│   ├── gimbal_controllers.yaml ........... 云台控制器参数
│   ├── camera_info.yaml .................. 相机内参
│   ├── lidar_config.yaml ................. 激光雷达参数
│   └── rviz_config.rviz .................. RViz可视化配置
│
├── worlds/ ................................ Gazebo世界
│   └── gimbal_sim.world .................. 仿真场景
│
├── launch/ ................................ 启动脚本
│   ├── gazebo_sim.launch.py .............. 启动Gazebo
│   └── full_system.launch.py ............. 启动完整系统
│
├── README.md .............................. 使用文档
├── INTEGRATION_GUIDE.md ................... 集成指南
├── TUNING_GUIDE.md ........................ 调优指南
├── setup.sh ............................... 快速启动脚本
└── test_simulator.sh ...................... 测试脚本
```

## 硬件模型规格

### 云台（Gimbal Platform）

**尺寸**:
- 基座：200×200×50 mm
- Pan轴：半径30mm，长100mm
- Tilt轴：半径30mm，长100mm

**传感器**:
- RGB相机：960×720，焦距1470
- Lidar：16线，360°扫描，0.27-25m范围

**控制**:
- Pan：±π rad，max 2.0 rad/s
- Tilt：-π/2～π/2 rad，max 2.0 rad/s

### 目标（QR Code Target）

**规格**:
- 100×100×100 mm立方体
- 可移动，支持多种轨迹

**支持轨迹**:
- 圆形：固定半径圆周运动
- 8字形：8字形轨迹
- 螺旋上升：逐渐上升的螺旋轨迹

## 通信接口

### 发布话题

| 话题 | 类型 | 频率 | 来源 |
|------|------|------|------|
| `/predicted_trajectory` | DetectionArray | 10Hz | target_tracker_node |
| `/gimbal_state` | Float32MultiArray | 100Hz | gimbal_controller_node |
| `/camera/image_rect` | Image | 30Hz | Gazebo Camera |
| `/camera_info` | CameraInfo | 1Hz | gazebo_bridge_node |
| `/lidar_points` | PointCloud2 | 10Hz | Gazebo Lidar |

### 订阅话题

| 话题 | 类型 | 来源 |
|------|------|------|
| `/gimbal_command` | TwistStamped | mpc_view_planner_node |
| `/joint_states` | JointState | Gazebo |
| `/gazebo/model_states` | ModelStates | Gazebo |

## 性能指标

### 计算性能

- **Gazebo模拟频率**: 1000 Hz
- **传感器发布**: 30 Hz (Camera), 10 Hz (Lidar)
- **MPC规划延迟**: 5-10 ms
- **云台控制频率**: 100 Hz
- **总系统延迟**: 30-50 ms

### 资源消耗

- **CPU**: 15-25%（i7-10700K）
- **内存**: 800-1000 MB
- **磁盘**: ~50 MB

## 扩展点

### 短期扩展
- [ ] 集成真实YOLO推理
- [ ] 实现PID对照控制
- [ ] 多目标跟踪
- [ ] 实验数据自动保存

### 中期扩展
- [ ] 无人机仿真模型替换
- [ ] 风场/动态环境模拟
- [ ] 目标跟踪精度评估
- [ ] 轨迹预测算法对比

### 长期展望
- [ ] 仿真-实物对比验证
- [ ] 参数优化框架
- [ ] 强化学习策略训练
- [ ] 数据集生成管道

## 参考资源

- [ROS 2 官方文档](https://docs.ros.org/)
- [Gazebo 用户指南](http://gazebosim.org/tutorials)
- [URDF 参考](http://wiki.ros.org/urdf)
- [MPC 理论](https://en.wikipedia.org/wiki/Model_predictive_control)

---

版本：1.0.0
最后更新：2026-04-14
