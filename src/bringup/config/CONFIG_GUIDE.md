# Config 配置文件说明

## 目录结构

```
config/
├── reality/                    # 现实环境配置
│   ├── all_in_one_param.yaml   # 传感器综合参数
│   ├── lidar_config.json       # LiDAR配置（JSON格式）
│   └── nav2_params.yaml        # 导航栈参数
└── simulation/                 # 仿真环境配置
    ├── nav2_params.yaml        # 基础导航栈参数
    ├── nav2_params_mppi.yaml   # MPPI规划器参数
    └── nav2_params_smoother.yaml # 路径平滑器参数
```

## 现实环境配置 (Reality)

### all_in_one_param.yaml
系统的综合参数配置，包括：
- 传感器驱动参数
- 里程计参数
- 同步参数
- 其他硬件相关参数

### lidar_config.json
LiDAR硬件配置，包括：
- 扫描线数
- 角度分辨率
- 时间戳格式
- 盲区设置

### nav2_params.yaml
导航相关参数，包括：
- SLAM配置
- 点云处理参数
- IMU参数
- 地图构建参数

## 仿真环境配置 (Simulation)

### nav2_params.yaml
基础导航栈参数，适合仿真环境，包括：
- 点云处理参数（模拟LiDAR）
- IMU参数
- 地图参数

### nav2_params_mppi.yaml
MPPI（模型预测路径积分）规划器参数：
- 轨迹采样数
- 成本函数权重
- 优化步数

### nav2_params_smoother.yaml
路径平滑器参数：
- 平滑算法选择
- 平滑程度
- 约束条件

## 使用指南

### 在启动文件中加载配置

```python
from launch.substitutions import LaunchConfiguration

# 加载配置文件
params_file = LaunchConfiguration("params_file")

# 在启动参数中声明
DeclareLaunchArgument(
    "params_file",
    default_value=os.path.join(
        bringup_dir, "config", "reality", "nav2_params.yaml"
    ),
    description="Full path to config file"
)
```

### 修改配置参数

1. **现实环境**: 编辑 `config/reality/` 中的文件
2. **仿真环境**: 编辑 `config/simulation/` 中的文件

### 配置参数对比

| 参数 | 现实 | 仿真 | 说明 |
|------|------|------|------|
| lid_topic | livox/lidar | velodyne_points | LiDAR话题名 |
| lidar_type | 1 (Livox) | 2 (Velodyne) | LiDAR类型 |
| use_sim_time | false | true | 是否使用仿真时间 |
| init_map_size | 10 | 10 | 初始地图大小 |
| filter_size_map | 0.2 | 0.2 | 地图滤波大小 |

## 常见配置修改

### 修改LiDAR话题

在 `nav2_params.yaml` 中修改：
```yaml
common:
  lid_topic: "your_lidar_topic"  # 改为你的话题名
```

### 修改IMU话题

```yaml
common:
  imu_topic: "your_imu_topic"  # 改为你的IMU话题名
```

### 调整地图滤波精度

较小的值 = 更精细的地图（更耗CPU）：
```yaml
mapping:
  filter_size_map: 0.1  # 改为需要的值
```

### 启用先验地图

```yaml
prior_pcd:
  enable: True
  # 注：launch文件中会自动设置路径
```

## 环境变量配置

如果需要在运行时切换配置，可以通过launch参数：

```bash
# 使用自定义配置文件
ros2 launch bringup bringup_main.py use_sim:=false \
  params_file:=$(pwd)/my_config.yaml
```

## 验证配置

启动系统后，可以通过以下命令检查配置是否正确加载：

```bash
# 检查节点参数
ros2 param get /node_name parameter_name

# 列出所有节点参数
ros2 param list /node_name

# 查看配置的话题
ros2 topic list
```

## 注意事项

1. **配置文件编码**: 确保所有YAML文件使用UTF-8编码
2. **缩进**: YAML对缩进敏感，使用空格而不是制表符
3. **类型检查**: 注意参数类型（数字、布尔值、字符串）
4. **话题名称**: 确保配置中的话题名与实际发布的话题一致
5. **参数范围**: 某些参数有推荐范围，超出范围可能导致不良行为

---

**最后更新**: 2026-04-28
