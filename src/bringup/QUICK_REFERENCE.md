# 🚀 Bringup 快速参考卡

## 最常用命令

### 启动仿真
```bash
# 基础仿真
ros2 launch bringup bringup_main.py use_sim:=true

# 仿真 + SLAM建图
ros2 launch bringup bringup_main.py use_sim:=true slam:=true

# 指定场景
ros2 launch bringup bringup_main.py use_sim:=true world:=rmuc_2025
```

### 启动实机
```bash
# 基础实机导航
ros2 launch bringup bringup_main.py use_sim:=false

# 实机 + SLAM建图
ros2 launch bringup bringup_main.py use_sim:=false slam:=true

# 关闭RViz
ros2 launch bringup bringup_main.py use_sim:=false use_rviz:=false
```

## 目录快速定位

| 需求 | 文件位置 |
|------|---------|
| 想了解包的整体结构 | → [PACKAGE_STRUCTURE.md](PACKAGE_STRUCTURE.md) |
| 想了解各启动文件的用途 | → [launch/LAUNCH_GUIDE.md](launch/LAUNCH_GUIDE.md) |
| 想了解如何修改配置 | → [config/CONFIG_GUIDE.md](config/CONFIG_GUIDE.md) |
| 想了解依赖关系 | → [DEPENDENCY_NOTE.md](DEPENDENCY_NOTE.md) |
| 想知道最近的改动 | → [CHANGELOG.md](CHANGELOG.md) |

## 文件分类速查

### 通用启动文件（Common）
```
launch/common/
├── bringup_launch.py          # 完整系统启动
├── communication.launch.py    # MCU通信
├── navigation_launch.py       # 导航栈
├── slam_launch.py             # SLAM系统
├── localization_launch.py     # 定位
├── rviz_launch.py             # 可视化
└── ...
```

### 现实启动文件（Reality）
```
launch/reality/
├── rm_navigation_reality_launch.py      # 完整
├── rm_navigation_reality_launch_nm.py   # 无地图管理
└── rm_navigation_reality_launch_pure.py # 仅导航
```

### 仿真启动文件（Simulation）
```
launch/simulation/
├── rm_navigation_simulation_launch.py      # 单机
├── rm_navigation_simulation_launch_dec.py  # 装饰版
└── rm_multi_navigation_simulation_launch.py # 多机
```

## 常用参数

```bash
# 完整参数列表
ros2 launch bringup bringup_main.py --show-args

# 常用组合
use_sim:=true|false          # 环境选择 (必需)
slam:=true|false             # 启用SLAM
world:=rmuc_2025             # 仿真场景
use_rviz:=true|false         # 启用RViz
autostart:=true|false        # 自动启动
namespace:=""                # 机器人命名空间
```

## 故障排除

| 问题 | 解决 |
|------|------|
| Launch file not found | `colcon build --packages-select bringup` |
| Topic not connecting | `ros2 topic list` 检查话题名称 |
| Parameter not loading | `ros2 param list` 查看实际参数 |
| RViz无数据 | 检查配置文件中的话题名是否匹配 |

## 编译与安装

```bash
# 构建
colcon build --packages-select bringup

# 加载环境
source install/setup.bash

# 验证
ros2 pkg list | grep bringup
```

## 默认配置路径

| 类型 | 路径 |
|------|------|
| 现实导航参数 | `config/reality/nav2_params.yaml` |
| 仿真导航参数 | `config/simulation/nav2_params.yaml` |
| 现实传感器参数 | `config/reality/all_in_one_param.yaml` |
| 仿真多机参数 | `config/simulation/nav2_params_mppi.yaml` |

## 关键文件

- **主启动**: `launch/bringup_main.py` ✨ 推荐使用
- **现实启动**: `launch/reality/rm_navigation_reality_launch.py`
- **仿真启动**: `launch/simulation/rm_navigation_simulation_launch.py`
- **现实参数**: `config/reality/nav2_params.yaml`
- **仿真参数**: `config/simulation/nav2_params.yaml`

## 快速复制-粘贴

### 最小启动
```bash
# 仿真
ros2 launch bringup bringup_main.py use_sim:=true

# 实机
ros2 launch bringup bringup_main.py use_sim:=false
```

### 调试启动
```bash
# 查看启动描述
ros2 launch bringup bringup_main.py --show-description

# 显示所有参数
ros2 launch bringup bringup_main.py --show-args

# 详细输出
ros2 launch bringup bringup_main.py use_sim:=true -v
```

### 自定义配置
```bash
# 使用自己的参数文件
ros2 launch bringup bringup_main.py use_sim:=false \
  params_file:=$(pwd)/my_params.yaml
```

## 环境检查

```bash
# 检查包是否安装
ros2 pkg list | grep bringup

# 检查依赖是否满足
ros2 pkg executables | grep -E "bringup|nav2|fast_lio"

# 检查配置文件
ls -la src/bringup/config/reality/
ls -la src/bringup/config/simulation/

# 检查话题
ros2 topic list | grep -E "lidar|imu|nav"
```

---

**提示**: 将此卡片加入书签以快速查询！

**最后更新**: 2026-04-28
