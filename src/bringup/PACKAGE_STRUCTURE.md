# Bringup 功能包完整指南

## 📚 快速导航

- **快速开始**: 见 [README_bringup.md](README_bringup.md)
- **快速参考**: 见 [QUICK_REFERENCE.md](QUICK_REFERENCE.md) ⭐ 常用命令速查
- **启动文件详情**: 见 [launch/LAUNCH_GUIDE.md](launch/LAUNCH_GUIDE.md)
- **配置文件详情**: 见 [config/CONFIG_GUIDE.md](config/CONFIG_GUIDE.md)
- **依赖说明**: 见 [DEPENDENCY_NOTE.md](DEPENDENCY_NOTE.md)
- **变更日志**: 见 [CHANGELOG.md](CHANGELOG.md)

---

## 📁 完整目录结构

```
bringup/
├── launch/
│   ├── bringup_main.py                      # ★ 主启动文件（推荐）
│   ├── LAUNCH_GUIDE.md                      # 启动文件完整指南
│   │
│   ├── common/                              # 通用模块
│   │   ├── bringup_launch.py
│   │   ├── bringup_launch_test.py
│   │   ├── buaa_sentry_publisher_launch.py
│   │   ├── communication.launch.py
│   │   ├── joy_teleop_launch.py
│   │   ├── localization_launch.py
│   │   ├── localization_launch_test.py
│   │   ├── navigation_launch.py
│   │   ├── robot_state_publisher_launch.py
│   │   ├── rviz_launch.py
│   │   ├── slam_launch.py
│   │   └── static_tf_publisher_launch.py
│   │
│   ├── reality/                             # 现实环境特定启动
│   │   ├── rm_navigation_reality_launch.py
│   │   ├── rm_navigation_reality_launch_nm.py
│   │   └── rm_navigation_reality_launch_pure.py
│   │
│   └── simulation/                          # 仿真环境特定启动
│       ├── rm_multi_navigation_simulation_launch.py
│       ├── rm_navigation_simulation_launch.py
│       └── rm_navigation_simulation_launch_dec.py
│
├── config/
│   ├── CONFIG_GUIDE.md                      # 配置文件完整指南
│   │
│   ├── reality/                             # 现实环境配置
│   │   ├── all_in_one_param.yaml
│   │   ├── lidar_config.json
│   │   └── nav2_params.yaml
│   │
│   └── simulation/                          # 仿真环境配置
│       ├── nav2_params.yaml
│       ├── nav2_params_mppi.yaml
│       └── nav2_params_smoother.yaml
│
├── rviz/                                    # RViz配置
│   └── (rviz config files)
│
├── README_bringup.md                        # 快速开始指南
├── PACKAGE_STRUCTURE.md                     # 本文件
├── package.xml                              # 包定义
└── CMakeLists.txt                           # 构建配置
```

---

## 🎯 核心概念

### 仿真 vs 现实

bringup包将所有启动文件和配置分为三类：

| 分类 | 用途 | 文件位置 | 加载配置 |
|------|------|---------|---------|
| **Common** | 通用模块 | `launch/common/` | 无环境特定配置 |
| **Reality** | 实机启动 | `launch/reality/` | `config/reality/` |
| **Simulation** | 仿真启动 | `launch/simulation/` | `config/simulation/` |

### 主启动文件的作用

`bringup_main.py` 是统一入口：
- 接收 `use_sim` 参数判断环境
- 自动加载对应的启动文件和配置
- 统一参数接口，降低使用复杂度

```bash
# 自动选择仿真
ros2 launch bringup bringup_main.py use_sim:=true

# 自动选择现实
ros2 launch bringup bringup_main.py use_sim:=false
```

---

## 🚀 使用场景

### 场景1: 仿真环境开发

```bash
# 启动Gazebo仿真 + SLAM建图
ros2 launch bringup bringup_main.py \
  use_sim:=true \
  slam:=true \
  world:=rmuc_2025
```

**启动链路**:
```
bringup_main.py 
  → rm_navigation_simulation_launch.py
    → common/slam_launch.py
    → common/navigation_launch.py
    → common/rviz_launch.py
```

### 场景2: 实机部署

```bash
# 启动实机 + 导航
ros2 launch bringup bringup_main.py \
  use_sim:=false \
  slam:=false
```

**启动链路**:
```
bringup_main.py 
  → rm_navigation_reality_launch.py
    → common/communication.launch.py
    → common/localization_launch.py
    → common/navigation_launch.py
    → common/rviz_launch.py
```

### 场景3: 多机仿真

```bash
# 启动多机仿真协作
ros2 launch bringup rm_multi_navigation_simulation_launch.py
```

### 场景4: 最小化启动

```bash
# 仅启动纯导航模块
ros2 launch bringup rm_navigation_reality_launch_pure.py
```

---

## 🔧 配置管理

### 配置继承关系

```
config/
├── reality/
│   ├── all_in_one_param.yaml         # 完整系统参数
│   ├── lidar_config.json             # LiDAR硬件配置
│   └── nav2_params.yaml              # 导航参数（基于lidar_config）
│
└── simulation/
    ├── nav2_params.yaml              # 基础导航参数
    ├── nav2_params_mppi.yaml         # 扩展：MPPI规划
    └── nav2_params_smoother.yaml     # 扩展：路径平滑
```

### 配置加载流程

1. 启动文件声明默认配置路径
2. 通过 `params_file` 参数加载指定配置
3. Nav2 RewrittenYaml 处理参数替换
4. 参数发送到各节点

---

## 📊 启动流程图

### 仿真启动流程

```
                    bringup_main.py
                          ↓
        [use_sim:=true 判断] → simulation/
                          ↓
      rm_navigation_simulation_launch.py
                          ↓
         ┌────────────────┬────────────┐
         ↓                ↓            ↓
      Gazebo      SLAM(fast_lio)   Nav2栈
         ↓                ↓            ↓
    点云仿真      地图构建      路径规划
         └────────────────┬────────────┘
                          ↓
                      RViz可视化
```

### 现实启动流程

```
                    bringup_main.py
                          ↓
        [use_sim:=false 判断] → reality/
                          ↓
       rm_navigation_reality_launch.py
                          ↓
         ┌────────────────┬────────────┐
         ↓                ↓            ↓
   传感器驱动    SLAM(fast_lio)   Nav2栈
         ↓                ↓            ↓
    LiDAR/IMU    地图构建      路径规划
         └────────────────┬────────────┘
                          ↓
                      RViz可视化
```

---

## 🎓 学习路径

### 初级用户
1. 阅读 [README_bringup.md](README_bringup.md)
2. 使用 `bringup_main.py` 启动系统
3. 尝试基本参数修改

### 中级用户
1. 阅读 [launch/LAUNCH_GUIDE.md](launch/LAUNCH_GUIDE.md)
2. 了解各启动文件的职责
3. 根据需求选择合适的启动组合

### 高级用户
1. 阅读 [config/CONFIG_GUIDE.md](config/CONFIG_GUIDE.md)
2. 修改配置参数优化性能
3. 创建自定义启动组合

---

## 🛠️ 开发指南

### 添加新的启动文件

1. **判断类型**: Common / Reality / Simulation
2. **放置位置**: `launch/{type}/` 目录
3. **命名规范**: 使用下划线分隔 (`module_function_launch.py`)
4. **配置文件**: 如需特定配置，放到 `config/{type}/`

### 修改启动逻辑

编辑对应的启动文件或 `bringup_main.py`:

```python
# 在 bringup_main.py 中添加新参数
declare_my_param_cmd = DeclareLaunchArgument(
    "my_param",
    default_value="default_value",
    description="Parameter description"
)
```

### 添加新配置

1. 在 `config/reality/` 或 `config/simulation/` 创建配置文件
2. 在启动文件中声明配置路径
3. 更新 [config/CONFIG_GUIDE.md](config/CONFIG_GUIDE.md)

---

## 📋 检查清单

### 部署前检查

- [ ] 所有依赖包已安装（见 `package.xml`）
- [ ] 配置文件中话题名称与实际发布话题一致
- [ ] 硬件驱动已启动（若使用现实环境）
- [ ] Gazebo可执行（若使用仿真环境）
- [ ] RViz配置文件存在（若使用可视化）

### 启动前检查

- [ ] ROS2 master已运行
- [ ] 所有节点依赖已满足
- [ ] 网络连接正常
- [ ] 磁盘空间充足

### 启动后检查

- [ ] 各节点成功加载
- [ ] 话题正常发布
- [ ] RViz可正常接收数据
- [ ] 无错误日志输出

---

## 🐛 常见问题

### Q: 如何切换仿真和现实?
**A**: 使用 `bringup_main.py` 的 `use_sim` 参数:
```bash
ros2 launch bringup bringup_main.py use_sim:=true   # 仿真
ros2 launch bringup bringup_main.py use_sim:=false  # 现实
```

### Q: 如何只启动导航不启动SLAM?
**A**: 
```bash
ros2 launch bringup bringup_main.py slam:=false
```

### Q: 如何使用自定义配置文件?
**A**:
```bash
ros2 launch bringup bringup_main.py \
  params_file:=/path/to/my_params.yaml
```

### Q: 各环境的配置有什么区别?
**A**: 见 [config/CONFIG_GUIDE.md](config/CONFIG_GUIDE.md)

### Q: 如何调试启动文件?
**A**:
```bash
# 显示启动描述
ros2 launch bringup bringup_main.py --show-description

# 显示所有参数
ros2 launch bringup bringup_main.py --show-args

# 详细输出
ros2 launch bringup bringup_main.py use_sim:=true -v
```

---

## 📞 获取帮助

### 文件导航
- **快速开始**: [README_bringup.md](README_bringup.md)
- **启动文件**: [launch/LAUNCH_GUIDE.md](launch/LAUNCH_GUIDE.md)
- **配置详情**: [config/CONFIG_GUIDE.md](config/CONFIG_GUIDE.md)

### 相关文档
- [ROS2 Launch文档](https://docs.ros.org/en/humble/Concepts/Intermediate/Launch-system.html)
- [Nav2文档](https://docs.nav2.org/)
- [Gazebo文档](http://gazebosim.org/docs)

---

## 版本信息

- **版本**: 0.1.0
- **最后更新**: 2026-04-28
- **维护者**: Lihan Chen
- **许可证**: Apache-2.0

---

**祝你使用愉快！** 🎉

如有问题或建议，请提交Issue或Pull Request。
