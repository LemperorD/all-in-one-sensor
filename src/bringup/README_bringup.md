# Bringup 功能包使用指南

**相关文档**: [完整包结构说明](PACKAGE_STRUCTURE.md) | [启动文件详情](launch/LAUNCH_GUIDE.md) | [配置文件详情](config/CONFIG_GUIDE.md) | [依赖说明](DEPENDENCY_NOTE.md)

## 📋 功能包结构

```
bringup/
├── launch/
│   ├── common/                    # 通用启动文件
│   │   ├── bringup_launch.py
│   │   ├── communication.launch.py
│   │   ├── navigation_launch.py
│   │   ├── slam_launch.py
│   │   ├── robot_state_publisher_launch.py
│   │   └── ...
│   ├── reality/                   # 现实环境启动文件
│   │   ├── rm_navigation_reality_launch.py
│   │   ├── rm_navigation_reality_launch_nm.py
│   │   └── rm_navigation_reality_launch_pure.py
│   ├── simulation/                # 仿真环境启动文件
│   │   ├── rm_navigation_simulation_launch.py
│   │   ├── rm_navigation_simulation_launch_dec.py
│   │   └── rm_multi_navigation_simulation_launch.py
│   └── bringup_main.py            # 主启动文件（统一入口）
├── config/
│   ├── reality/                   # 现实环境配置
│   │   ├── all_in_one_param.yaml
│   │   └── lidar_config.json
│   └── simulation/                # 仿真环境配置
│       ├── nav2_params.yaml
│       ├── nav2_params_mppi.yaml
│       └── nav2_params_smoother.yaml
├── rviz/                          # RViz配置文件
├── package.xml
└── CMakeLists.txt
```

## 🚀 快速开始

### 仿真环境启动

```bash
# 最简单的启动方式
ros2 launch bringup bringup_main.py use_sim:=true

# 指定特定场景
ros2 launch bringup bringup_main.py use_sim:=true world:=rmuc_2025

# 启用SLAM
ros2 launch bringup bringup_main.py use_sim:=true slam:=true
```

### 现实环境启动

```bash
# 最简单的启动方式
ros2 launch bringup bringup_main.py use_sim:=false

# 启用SLAM
ros2 launch bringup bringup_main.py use_sim:=false slam:=true

# 不启用RViz
ros2 launch bringup bringup_main.py use_sim:=false use_rviz:=false
```

## 📝 启动参数说明

### bringup_main.py 参数

| 参数 | 默认值 | 说明 | 可选值 |
|------|-------|------|--------|
| `use_sim` | false | 是否使用仿真环境 | true/false |
| `namespace` | "" | 机器人命名空间 | 字符串 |
| `slam` | false | 是否启用SLAM | true/false |
| `world` | rmuc_2025 | 仿真场景名称 | rmul_2024/rmuc_2024/rmul_2025/rmuc_2025/rmuc_2026/rmul_2026 |
| `use_rviz` | true | 是否启用RViz | true/false |
| `autostart` | true | 是否自动启动导航栈 | true/false |

## 📦 各启动文件详细说明

### Common（通用启动文件）

| 文件 | 功能 |
|------|------|
| `bringup_launch.py` | 完整系统启动 |
| `communication.launch.py` | 与MCU通信启动 |
| `navigation_launch.py` | 导航栈启动 |
| `slam_launch.py` | SLAM启动 |
| `robot_state_publisher_launch.py` | 机器人状态发布 |
| `localization_launch.py` | 定位启动 |
| `rviz_launch.py` | RViz可视化启动 |

### Reality（现实环境启动文件）

| 文件 | 功能 |
|------|------|
| `rm_navigation_reality_launch.py` | 完整导航启动 |
| `rm_navigation_reality_launch_nm.py` | 导航启动（不含地图） |
| `rm_navigation_reality_launch_pure.py` | 纯导航启动 |

### Simulation（仿真环境启动文件）

| 文件 | 功能 |
|------|------|
| `rm_navigation_simulation_launch.py` | 完整导航启动 |
| `rm_navigation_simulation_launch_dec.py` | 导航启动（带装饰）|
| `rm_multi_navigation_simulation_launch.py` | 多机导航启动 |

## ⚙️ 配置文件说明

### Reality 配置

- `all_in_one_param.yaml` - 传感器和系统参数
- `lidar_config.json` - LiDAR参数配置

### Simulation 配置

- `nav2_params.yaml` - 基础导航参数
- `nav2_params_mppi.yaml` - MPPI规划器参数
- `nav2_params_smoother.yaml` - 平滑器参数

## 🛠️ 常见使用场景

### 场景1：启动现实机器人进行导航

```bash
ros2 launch bringup bringup_main.py use_sim:=false slam:=false
```

### 场景2：启动仿真进行算法测试

```bash
ros2 launch bringup bringup_main.py use_sim:=true world:=rmuc_2025 slam:=false
```

### 场景3：启动现实机器人进行SLAM建图

```bash
ros2 launch bringup bringup_main.py use_sim:=false slam:=true
```

### 场景4：启动仿真进行SLAM建图

```bash
ros2 launch bringup bringup_main.py use_sim:=true slam:=true world:=rmul_2025
```

## 📋 依赖包

- `nav2_*` - 导航相关包
- `fast_lio` - LiDAR-IMU里程计
- `yolo_ros` - 目标检测
- `simulator` - 仿真环境
- `serial_communication` - 串行通信
- `sync_board_ros2_driver` - 传感器同步
- `hik_camera_ros2_driver` - 相机驱动

## 🐛 故障排除

### 问题1：launch文件找不到

**原因**：bringup包未编译或路径错误

**解决**：
```bash
cd ~/all-in-one-sensor
colcon build --packages-select bringup
source install/setup.bash
```

### 问题2：配置文件路径错误

**原因**：config目录结构改变

**解决**：检查配置文件是否存在于 `config/reality/` 或 `config/simulation/`

### 问题3：多机器人命名空间冲突

**解决**：使用namespace参数隔离
```bash
ros2 launch bringup bringup_main.py namespace:=robot1 use_sim:=true
```

## 📚 参考资源

- [ROS2 Launch文档](https://docs.ros.org/en/humble/Concepts/Intermediate/Launch-system.html)
- [Nav2文档](https://docs.nav2.org/)
- [Gazebo仿真指南](http://gazebosim.org/docs)

---

**最后更新**: 2026-04-28  
**维护者**: Lihan Chen
