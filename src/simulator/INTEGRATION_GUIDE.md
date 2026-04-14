# 仿真平台与实际模块集成指南

## 系统架构概览

本仿真平台设计用于无缝集成以下模块：
- **lidar_detector**: 传感器融合检测
- **mpc_gimbal_planner**: MPC视角规划
- **yolo_ros**: YOLO目标检测

## 话题映射和数据流

### 1. 目标检测数据流

```
┌─────────────────────────────────────────────────────┐
│                  Gazebo Simulator                     │
├─────────────────────────────────────────────────────┤
│  Camera (rgb/image_rect)  →  │  Lidar (point_cloud2) │
└─────────────────────────────────────────────────────┘
           ↓                              ↓
    YOLO Detection            DBSCAN Clustering
           ↓                              ↓
    └──────────── Sensor Fusion ────────────┘
                  ↓
        FusedDetectionArray
                  ↓
        yolo_msgs/DetectionArray
```

### 2. MPC规划流程

```
FusedDetectionArray (目标检测)
          ↓
predicted_trajectory (目标轨迹预测)
          ↓
mpc_view_planner_node (MPC优化)
          ↓
gimbal_command (TwistStamped)
          ↓
gimbal_controller_node (执行控制)
          ↓
Gazebo云台更新
```

## 话题规范

### 必需话题

| 话题 | 类型 | 方向 | 来源 | 目标 |
|------|------|------|------|------|
| `/predicted_trajectory` | DetectionArray | Pub | target_tracker_node | mpc_view_planner_node |
| `/gimbal_command` | TwistStamped | Sub | mpc_view_planner_node | gimbal_controller_node |
| `/camera/image_rect` | Image | Pub | Gazebo Camera | yolo_ros |
| `/lidar_points` | PointCloud2 | Pub | Gazebo Lidar | lidar_detector |
| `/gimbal_state` | Float32MultiArray | Pub | gimbal_controller_node | Monitoring |

### 可选话题

| 话题 | 类型 | 方向 | 用途 |
|------|------|------|------|
| `/sim_target_detection` | DetectionArray | Pub | 调试/验证 |
| `/camera_info` | CameraInfo | Pub | 相机标定 |
| `/fused_detections` | DetectionArray | Pub | 融合结果验证 |

## 启动顺序

### 方法 1：完全自动化启动（推荐）

```bash
# 一条命令启动所有组件
ros2 launch simulator full_system.launch.py \
    trajectory_type:=circular \
    trajectory_radius:=5.0
```

### 方法 2：分步启动（调试用）

**终端 1：启动Gazebo仿真**
```bash
ros2 launch simulator gazebo_sim.launch.py
```

**终端 2：启动云台控制**
```bash
ros2 run simulator gimbal_controller_node
```

**终端 3：启动目标跟踪**
```bash
ros2 run simulator target_tracker_node \
    --ros-args \
    -p trajectory_type:=circular \
    -p trajectory_radius:=5.0 \
    -p trajectory_height:=2.0
```

**终端 4：启动Gazebo桥接**
```bash
ros2 run simulator gazebo_bridge_node
```

**终端 5：启动MPC规划器**
```bash
ros2 run mpc_gimbal_planner mpc_view_planner_node \
    --ros-args \
    -p mpc_horizon:=10 \
    -p planning_period:=0.1 \
    -p w_tracking:=1.0 \
    -p w_smoothness:=0.5 \
    -p w_control:=0.2
```

**终端 6：启动传感器融合**
```bash
ros2 run lidar_detector sensor_fusion_node \
    --ros-args \
    -p dbscan_eps:=0.1 \
    -p dbscan_min_pts:=5 \
    -p camera_fx:=1470.0 \
    -p camera_fy:=1470.0 \
    -p camera_cx:=480.0 \
    -p camera_cy:=360.0
```

## 参数调优指南

### 1. MPC规划器参数

| 参数 | 范围 | 建议值 | 说明 |
|------|------|--------|------|
| `mpc_horizon` | 3-20 | 10 | 预测步数越多越精确，但计算时间增加 |
| `planning_period` | 0.05-0.2 | 0.1 | 规划周期，应小于0.1s实时性要求 |
| `w_tracking` | 0.1-10 | 1.0 | 跟踪误差权重，越大跟踪越紧 |
| `w_smoothness` | 0.1-1.0 | 0.5 | 平滑度权重，减少关节震荡 |
| `w_control` | 0.1-1.0 | 0.2 | 控制量权重，降低执行器负荷 |
| `max_pan_rate` | 0.5-5.0 | 2.0 | 最大偏航速度 |
| `max_tilt_rate` | 0.5-5.0 | 2.0 | 最大俯仰速度 |

### 2. 传感器融合参数

| 参数 | 范围 | 建议值 | 说明 |
|------|------|--------|------|
| `dbscan_eps` | 0.02-0.5 | 0.1 | 点云聚类半径 |
| `dbscan_min_pts` | 3-20 | 5 | 聚类最小点数 |
| `iou_threshold` | 0.3-0.7 | 0.5 | IoU匹配阈值 |
| `camera_fx` | 1000-2000 | 1470 | 相机焦距x |
| `camera_fy` | 1000-2000 | 1470 | 相机焦距y |

### 3. 目标轨迹参数

| 参数 | 范围 | 建议值 | 说明 |
|------|------|--------|------|
| `trajectory_radius` | 2-20 | 5.0 | 目标轨迹半径(m) |
| `trajectory_height` | 0.5-10 | 2.0 | 目标飞行高度(m) |
| `trajectory_period` | 10-60 | 20.0 | 完整轨迹周期(s) |

## 监测和调试

### 1. 查看实时话题

```bash
# 监视云台状态
ros2 topic echo /gimbal_state

# 监视目标检测
ros2 topic echo /predicted_trajectory --once

# 监视云台命令
ros2 topic echo /gimbal_command
```

### 2. 性能分析

```bash
# 查看消息频率
ros2 topic hz /predicted_trajectory
ros2 topic hz /gimbal_command

# 计算平均延迟
ros2 topic delay /gimbal_command
```

### 3. 关节状态监测

```bash
# 查看关节实时状态
ros2 topic echo /joint_states

# 绘制关节角度
ros2 run rqt_plot rqt_plot /joint_states/position[0] /joint_states/position[1]
```

### 4. RViz可视化

```bash
# 启用多个显示：
# - Robot Model: 显示云台模型
# - TF: 显示坐标变换
# - PointCloud2: 显示激光雷达点云 (可选)
# - Grid: 显示网格
```

## 常见问题

### Q1: 云台不响应MPC命令

**原因**：
- gimbal_controller_node未运行
- gimbal_command话题未发布
- 云台关节限制

**解决方案**：
```bash
# 检查话题
ros2 topic list | grep gimbal_command

# 检查节点运行
ros2 node list | grep gimbal_controller

# 验证关节状态
ros2 topic echo /joint_states
```

### Q2: 传感器融合失败

**原因**：
- 相机和激光雷达不同步
- 参数配置不当
- 点云和检测框不匹配

**解决方案**：
```bash
# 增大同步窗口
# 调整dbscan_eps参数
# 验证相机内参

ros2 param set /sensor_fusion_node dbscan_eps 0.15
```

### Q3: MPC规划频率低

**原因**：
- mpc_horizon过大
- 计算能力不足
- 其他进程占用CPU

**解决方案**：
```bash
# 减小预测步数
ros2 param set /mpc_view_planner_node mpc_horizon 5

# 监测CPU使用
top -p $(pgrep mpc_view_planner_node)
```

## 性能基准

### 预期指标（开发机 i7-10700K）

| 指标 | 值 |
|------|-----|
| Gazebo仿真频率 | 1000 Hz |
| 传感器发布频率 | 30 Hz (Camera), 10 Hz (Lidar) |
| MPC规划延迟 | 5-10 ms |
| 云台控制频率 | 100 Hz |
| 总系统延迟 | 30-50 ms |

## 下一步工作

1. **集成YOLO检测**：替换仿真检测为真实YOLO推理
2. **添加PID对照**：实现PID视觉伺服控制进行对比
3. **数据记录**：自动记录实验日志和轨迹数据
4. **可视化增强**：添加更详细的RViz显示配置
5. **无人机仿真**：替换QR码为无人机3D模型

## 参考资源

- [ROS 2官方文档](https://docs.ros.org/)
- [Gazebo使用指南](http://gazebosim.org/tutorials)
- [URDF教程](http://wiki.ros.org/urdf/Tutorials)
- [TF变换库](http://wiki.ros.org/tf2)

---

**最后更新**: 2026-04-14
**版本**: 1.0.0
**维护者**: ld
