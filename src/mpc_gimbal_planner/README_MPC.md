# MPC Gimbal View Planner

## 概述
基于模型预测控制(MPC)的主动视角规划模块，用于自动跟踪预测的目标轨迹并控制云台(PTZ)的转角。

## 核心特性

### 三项代价函数
1. **视角追踪项** (w_tracking): 最小化当前视角与预测目标的角度偏差
   - 确保目标时刻在视野内

2. **视角平滑项** (w_smoothness): 约束云台速度变化
   - 生成平滑的运动轨迹，避免抖动

3. **防止控制剧烈项** (w_control): 约束云台加速度
   - 防止突变的控制输入，保护云台硬件

### 约束条件
- 泛角速率限制: `max_pan_rate`, `max_tilt_rate`
- 泛角加速度限制: `max_pan_accel`, `max_tilt_accel`

## 数据结构

### TrajectoryPoint (IMMKF预测输出)
```cpp
struct TrajectoryPoint {
    double timestamp;           // 时间戳
    double x, y, z;            // 3D位置
    double vx, vy, vz;         // 3D速度
    double confidence;         // 预测置信度
};
```

### GimbalCommand (云台指令)
```cpp
struct GimbalCommand {
    double pan;                // 泛角 (弧度)
    double tilt;               // 俯角 (弧度)
    double pan_rate;           // 泛角速率 (rad/s)
    double tilt_rate;          // 俯角速率 (rad/s)
    double timestamp;          // 命令时间戳
};
```

## MPC求解原理

### 状态向量
- `x = [pan_0, tilt_0, pan_1, tilt_1, ..., pan_N, tilt_N]`
- N = 预测步长 - 1

### 优化问题
```
minimize: J = w_track * Σ||u_k - u_desired_k||²
            + w_smooth * Σ||u_k - u_{k-1}||²
            + w_control * Σ||a_k||²
```

其中 `a_k = (u_k - u_{k-1}) - (u_{k-1} - u_{k-2})` 是二阶差分（加速度）

### 求解方法
1. 构建Hessian矩阵和梯度向量
2. 使用Cholesky分解求解QP问题: `x* = -H^{-1} * g`
3. 提取第一步最优控制
4. 应用速率和加速度约束饱和

## ROS2节点

### 订阅话题
- `/predicted_trajectory` (yolo_msgs/DetectionArray)
  - IMMKF预测的目标轨迹

- `/gimbal_state` (std_msgs/Float32MultiArray)
  - 当前云台状态: [pan, tilt, ...]

### 发布话题
- `/gimbal_command` (geometry_msgs/TwistStamped)
  - angular: [tilt_rate, 0, pan_rate]
  - linear: [pan, tilt, 0] (备用绝对角度)

## 参数配置

### MPC参数
- `mpc_horizon`: 预测步长 (默认: 10)
- `mpc_dt`: 控制周期 (默认: 0.1s)
- `planning_period`: 规划频率 (默认: 0.1s)

### 代价函数权重
- `w_tracking`: 追踪权重 (默认: 1.0)
- `w_smoothness`: 平滑权重 (默认: 0.5)
- `w_control`: 控制权重 (默认: 0.2)

### 云台约束
- `max_pan_rate`: 最大泛角速率 (默认: 2.0 rad/s)
- `max_tilt_rate`: 最大俯角速率 (默认: 2.0 rad/s)
- `max_pan_accel`: 最大泛角加速度 (默认: 1.0 rad/s²)
- `max_tilt_accel`: 最大俯角加速度 (默认: 1.0 rad/s²)

### 相机参数
- `camera_fx`, `camera_fy`: 焦距 (默认: 1470)
- `camera_cx`, `camera_cy`: 主点 (默认: 480, 360)

## 编译方式

```bash
cd /home/ld/all-in-one-sensor
colcon build --packages-select mpc_gimbal_planner
```

## 运行方式

```bash
# 启动ROS2节点
ros2 run mpc_gimbal_planner mpc_view_planner_executable

# 或使用composition
ros2 run rclcpp_components component_container &
ros2 component load /ComponentManager mpc_gimbal_planner mpc_gimbal_planner::MPCViewPlannerNode
```

## 输出示例

云台指令中，如果目标在(5m, 3m, 2m):
- pan ≈ atan2(3, 5) ≈ 0.54 rad (30°)
- tilt ≈ atan2(-2, sqrt(25+9)) ≈ -0.31 rad (-18°)
- 云台会以受约束的速率向该方向转动

## 注意事项

1. **坐标系**:
   - pan: 绕z轴旋转，正值为左转
   - tilt: 绕y轴旋转，负值为下俯

2. **时间同步**:
   - 所有消息必须有正确的时间戳
   - 使用approximate time synchronizer处理时序偏差

3. **数值稳定性**:
   - Eigen库用于矩阵运算
   - 使用LDLT分解确保数值精度

4. **性能**:
   - 预测步长越长，计算量越大
   - 建议horizon = 10~20
