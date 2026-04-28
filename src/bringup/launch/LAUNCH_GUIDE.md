# Launch 启动文件说明

## 目录结构

```
launch/
├── bringup_main.py             # 主启动文件（统一入口）★ 推荐使用
├── common/                     # 通用启动文件
│   ├── bringup_launch.py
│   ├── bringup_launch_test.py
│   ├── buaa_sentry_publisher_launch.py
│   ├── communication.launch.py
│   ├── joy_teleop_launch.py
│   ├── localization_launch.py
│   ├── localization_launch_test.py
│   ├── navigation_launch.py
│   ├── robot_state_publisher_launch.py
│   ├── rviz_launch.py
│   ├── slam_launch.py
│   └── static_tf_publisher_launch.py
├── reality/                    # 现实环境启动文件
│   ├── rm_navigation_reality_launch.py
│   ├── rm_navigation_reality_launch_nm.py
│   └── rm_navigation_reality_launch_pure.py
└── simulation/                 # 仿真环境启动文件
    ├── rm_multi_navigation_simulation_launch.py
    ├── rm_navigation_simulation_launch.py
    └── rm_navigation_simulation_launch_dec.py
```

## 启动文件分类

### bringup_main.py（主启动文件）★ 推荐

**用途**: 统一入口，根据参数自动选择仿真或现实模式

**使用示例**:
```bash
# 仿真模式
ros2 launch bringup bringup_main.py use_sim:=true

# 现实模式
ros2 launch bringup bringup_main.py use_sim:=false
```

**主要参数**:
- `use_sim`: 仿真模式开关 (true/false)
- `slam`: SLAM启用开关 (true/false)
- `world`: 仿真场景选择
- `use_rviz`: RViz启用开关

---

## Common 通用启动文件

### bringup_launch.py
- **功能**: 完整系统启动
- **启动项**: 通信、导航、机器人状态发布
- **使用场景**: 需要完整系统

### communication.launch.py
- **功能**: 与MCU/嵌入式系统通信
- **启动项**: 串行通信驱动
- **使用场景**: 底层硬件控制

### navigation_launch.py
- **功能**: 导航栈启动
- **启动项**: Nav2框架
- **使用场景**: 路径规划、目标导航

### slam_launch.py
- **功能**: SLAM系统启动
- **启动项**: FAST-LIO、地图管理器
- **使用场景**: 建图和定位

### localization_launch.py
- **功能**: 定位系统启动
- **启动项**: 定位算法、地图匹配
- **使用场景**: 已知地图下定位

### robot_state_publisher_launch.py
- **功能**: 机器人状态发布
- **启动项**: TF发布、关节状态
- **使用场景**: 坐标系管理

### rviz_launch.py
- **功能**: RViz可视化启动
- **启动项**: RViz2与配置加载
- **使用场景**: 可视化调试

### joy_teleop_launch.py
- **功能**: 手柄遥控启动
- **启动项**: Joy驱动、遥控命令转换
- **使用场景**: 遥控操作

### static_tf_publisher_launch.py
- **功能**: 静态坐标系发布
- **启动项**: 静态TF框架
- **使用场景**: 固定坐标变换

### buaa_sentry_publisher_launch.py
- **功能**: 哨兵机器人发布器
- **启动项**: 特定机器人的发布节点
- **使用场景**: 哨兵特定功能

---

## Reality 现实环境启动文件

### rm_navigation_reality_launch.py
- **功能**: 完整实机导航启动
- **启动项**: 传感器驱动、SLAM、导航
- **使用场景**: 完整实机系统
- **特点**: 加载 `config/reality/` 配置

```bash
ros2 launch bringup rm_navigation_reality_launch.py
```

### rm_navigation_reality_launch_nm.py
- **功能**: 导航启动（不含地图管理器）
- **启动项**: 导航栈（最小化）
- **使用场景**: 已知地图、无需地图服务器
- **特点**: 轻量级启动

```bash
ros2 launch bringup rm_navigation_reality_launch_nm.py
```

### rm_navigation_reality_launch_pure.py
- **功能**: 纯导航启动
- **启动项**: 仅导航规划
- **使用场景**: 定位独立处理
- **特点**: 最小启动集

```bash
ros2 launch bringup rm_navigation_reality_launch_pure.py
```

---

## Simulation 仿真环境启动文件

### rm_navigation_simulation_launch.py
- **功能**: 完整仿真导航启动
- **启动项**: Gazebo、模拟传感器、导航
- **使用场景**: 单机仿真测试
- **特点**: 加载 `config/simulation/` 配置

```bash
ros2 launch bringup rm_navigation_simulation_launch.py world:=rmuc_2025
```

### rm_navigation_simulation_launch_dec.py
- **功能**: 装饰版仿真启动
- **启动项**: 仿真 + 装饰器/插件
- **使用场景**: 需要额外功能的仿真
- **特点**: 扩展功能

```bash
ros2 launch bringup rm_navigation_simulation_launch_dec.py
```

### rm_multi_navigation_simulation_launch.py
- **功能**: 多机仿真启动
- **启动项**: 多个机器人实例、Gazebo
- **使用场景**: 多机协作仿真
- **特点**: 需要指定namespace

```bash
ros2 launch bringup rm_multi_navigation_simulation_launch.py
```

---

## 使用指南

### 快速启动流程

#### 仿真模式
```bash
# 方法1：使用主启动文件（推荐）
ros2 launch bringup bringup_main.py use_sim:=true world:=rmuc_2025

# 方法2：直接启动仿真文件
ros2 launch bringup rm_navigation_simulation_launch.py world:=rmuc_2025
```

#### 现实模式
```bash
# 方法1：使用主启动文件（推荐）
ros2 launch bringup bringup_main.py use_sim:=false

# 方法2：直接启动现实文件
ros2 launch bringup rm_navigation_reality_launch.py
```

### 参数传递

```bash
# 传递多个参数
ros2 launch bringup bringup_main.py \
  use_sim:=true \
  world:=rmul_2025 \
  slam:=true \
  use_rviz:=true

# 使用自定义配置
ros2 launch bringup bringup_main.py \
  use_sim:=false \
  params_file:=/path/to/custom_params.yaml
```

### 调试启动

```bash
# 打印启动描述但不执行
ros2 launch bringup bringup_main.py --show-description use_sim:=true

# 显示所有可用参数
ros2 launch bringup bringup_main.py --show-args use_sim:=true

# 详细输出
ros2 launch bringup bringup_main.py use_sim:=true -v
```

---

## 选择启动文件的决策树

```
是否使用仿真？
├─ 是 → 需要多机吗？
│   ├─ 是 → rm_multi_navigation_simulation_launch.py
│   ├─ 否 → 需要装饰功能吗？
│   │   ├─ 是 → rm_navigation_simulation_launch_dec.py
│   │   └─ 否 → rm_navigation_simulation_launch.py
│   
└─ 否 (现实模式) → 需要地图服务器吗？
    ├─ 是 → rm_navigation_reality_launch.py
    ├─ 否，但需要基础导航 → rm_navigation_reality_launch_nm.py
    └─ 仅路径规划 → rm_navigation_reality_launch_pure.py
```

或者使用 **bringup_main.py**（自动处理上述逻辑）

---

## 常见组合启动方案

### 方案1：仿真算法开发
```bash
ros2 launch bringup bringup_main.py use_sim:=true slam:=false
```

### 方案2：SLAM建图（仿真）
```bash
ros2 launch bringup bringup_main.py use_sim:=true slam:=true
```

### 方案3：实机SLAM建图
```bash
ros2 launch bringup bringup_main.py use_sim:=false slam:=true
```

### 方案4：实机导航（使用已知地图）
```bash
ros2 launch bringup bringup_main.py use_sim:=false slam:=false
```

### 方案5：多机仿真协作
```bash
ros2 launch bringup rm_multi_navigation_simulation_launch.py
```

---

## 故障排除

### 问题1：启动文件找不到
**解决**:
```bash
colcon build --packages-select bringup
source install/setup.bash
```

### 问题2：参数未生效
**检查**:
```bash
# 查看实际加载的参数
ros2 param list /node_name
ros2 param get /node_name param_name
```

### 问题3：话题未连接
**调试**:
```bash
# 列出所有话题
ros2 topic list

# 检查话题类型和连接
ros2 topic info /topic_name
```

---

**最后更新**: 2026-04-28  
**维护者**: Lihan Chen
