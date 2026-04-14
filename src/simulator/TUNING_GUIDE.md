# Simulator 仿真平台参数调优指南

本文档提供了仿真平台的参数调优建议，帮助用户根据不同的实验需求进行优化。

## 快速参数调优

### 场景 1：提高跟踪精度

**MPC规划器参数**：
```bash
ros2 launch simulator full_system.launch.py \
    -p mpc_horizon:=15 \
    -p w_tracking:=2.0 \
    -p w_smoothness:=0.3 \
    -p w_control:=0.1
```

**说明**：
- 增加 `mpc_horizon` 到 15 步，获得更长的预测范围
- 提高 `w_tracking` 权重，重点关注跟踪误差
- 降低 `w_smoothness` 和 `w_control`，允许更激进的动作

### 场景 2：降低电机负荷（实物对接）

**云台控制参数**：
```bash
ros2 launch simulator full_system.launch.py \
    -p max_pan_rate:=1.5 \
    -p max_tilt_rate:=1.5 \
    -p w_tracking:=0.8 \
    -p w_smoothness:=0.7 \
    -p w_control:=0.3
```

**说明**：
- 限制最大角速度到 1.5 rad/s
- 增加 `w_smoothness` 和 `w_control` 平衡平滑性和能耗

### 场景 3：低延迟/实时性要求

```bash
ros2 launch simulator full_system.launch.py \
    -p mpc_horizon:=5 \
    -p planning_period:=0.05 \
    -p publish_rate:=20.0
```

**说明**：
- 减小预测步数到 5，加快计算
- 缩短规划周期到 50ms
- 提高发布频率到 20Hz

### 场景 4：复杂轨迹跟踪

```bash
ros2 launch simulator full_system.launch.py \
    trajectory_type:=spiral_up \
    trajectory_radius:=8.0 \
    trajectory_height:=3.0 \
    -p mpc_horizon:=12 \
    -p w_tracking:=1.5
```

## 逐步调优流程

### 1. 基准测试
```bash
# 使用默认参数进行基准测试
ros2 launch simulator full_system.launch.py

# 监视性能指标
ros2 topic hz /gimbal_command
ros2 topic delay /gimbal_command
```

### 2. 识别问题
查看以下指标：
- 跟踪误差是否太大？→ 增加 `w_tracking`
- 关节震荡？→ 增加 `w_smoothness`
- 计算延迟高？→ 减小 `mpc_horizon`
- 命令不稳定？→ 增加 `w_control`

### 3. 参数调整
```bash
# 监视效果
ros2 run rqt_plot rqt_plot \
    '/gimbal_state/data[0]' \
    '/gimbal_command/twist/linear/x'
```

### 4. 验证改进
```bash
# 对比调整前后的结果
# 记录日志用于离线分析
```

## 参数依赖关系

```
mpc_horizon ──→ 计算时间 ──→ planning_period
   ↓                           ↓
预测精度                    实时性

max_pan_rate ─→ 云台速度 ──→ w_smoothness
max_tilt_rate    ↓           ↓
            可控性        执行平滑度

w_tracking ──→ 跟踪精度
w_smoothness → 运动平滑度  ← 能耗
w_control ────→ 控制量限制
```

## 不同任务的推荐参数

### 任务 A：固定目标跟踪
```yaml
mpc_horizon: 8
planning_period: 0.1
w_tracking: 2.0
w_smoothness: 0.2
w_control: 0.1
max_pan_rate: 3.0
max_tilt_rate: 3.0
trajectory_type: "circular"
trajectory_radius: 3.0
trajectory_period: 15.0
```

### 任务 B：高速移动目标
```yaml
mpc_horizon: 12
planning_period: 0.05
w_tracking: 1.5
w_smoothness: 0.3
w_control: 0.2
max_pan_rate: 4.0
max_tilt_rate: 3.5
trajectory_type: "spiral_up"
trajectory_radius: 5.0
trajectory_period: 10.0
```

### 任务 C：长续航/低功耗
```yaml
mpc_horizon: 6
planning_period: 0.2
w_tracking: 1.0
w_smoothness: 1.0
w_control: 0.5
max_pan_rate: 1.5
max_tilt_rate: 1.5
trajectory_type: "circular"
trajectory_radius: 4.0
trajectory_period: 30.0
```

## 性能监测脚本

### 创建监测脚本 `monitor_sim.sh`
```bash
#!/bin/bash
echo "=== Simulator Performance Monitor ==="
echo ""
echo "Topic Frequencies:"
ros2 topic hz /gimbal_command & sleep 2
ros2 topic hz /predicted_trajectory & sleep 2
echo ""
echo "Message Delays:"
ros2 topic delay /gimbal_command & sleep 2
echo ""
echo "CPU Usage:"
top -b -n 1 | grep "mpc_view_planner"
```

### 数据分析脚本 `analyze_logs.py`
```python
#!/usr/bin/env python3
import json
import numpy as np
import matplotlib.pyplot as plt

def analyze_log(log_file):
    with open(log_file) as f:
        data = json.load(f)

    # 计算跟踪误差
    gimbal_states = np.array([[s['pan'], s['tilt']]
                              for s in data['gimbal_state']])

    # 导出为图表
    plt.figure(figsize=(12, 8))

    plt.subplot(2, 1, 1)
    plt.plot(gimbal_states[:, 0], label='Pan')
    plt.ylabel('Pan Angle (rad)')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(gimbal_states[:, 1], label='Tilt')
    plt.ylabel('Tilt Angle (rad)')
    plt.xlabel('Time Step')
    plt.legend()

    plt.tight_layout()
    plt.savefig('analysis.png')
    print("Analysis saved to analysis.png")

if __name__ == '__main__':
    import sys
    analyze_log(sys.argv[1])
```

## 常见调试命令

```bash
# 实时查看参数
ros2 param list
ros2 param get /mpc_view_planner_node mpc_horizon

# 动态修改参数
ros2 param set /gimbal_controller_node max_pan_rate 2.5

# 查看节点信息
ros2 node info /gimbal_controller_node

# 监视特定话题
ros2 topic echo /gimbal_state
ros2 topic echo /gimbal_command
ros2 topic echo /predicted_trajectory

# 计算话题同步状态
ros2 topic info /gimbal_command
```

## 调优检查清单

- [ ] 基准测试完成，记录基线指标
- [ ] 跟踪误差在可接受范围内
- [ ] 关节运动平滑无抖动
- [ ] 计算延迟 < 100ms
- [ ] CPU使用率 < 80%
- [ ] 所有话题正常发布
- [ ] 日志数据有效记录
- [ ] 参数配置已文档化

## 参考资源

- MPC理论: 《Model Predictive Control Theory and Practice》
- Gazebo调优: http://gazebosim.org/tutorials
- ROS参数管理: https://docs.ros.org/en/humble/Concept/Basic/About-Parameters.html

---

最后更新：2026-04-14
版本：1.0
