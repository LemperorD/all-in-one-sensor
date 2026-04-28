# ⚠️ 重要提示与依赖说明

## 包名称对应关系

### 当前包信息
- **包名**: `bringup`
- **位置**: `src/bringup/`
- **作用**: 系统启动配置管理和文件组织

### 依赖关系

```
bringup (当前包)
  ↓
  ├─ pb2025_nav_bringup (外部导航包)
  │   ├─ nav2_bringup
  │   ├─ fast_lio
  │   └─ ...其他导航依赖
  │
  ├─ simulator (本仓库)
  ├─ yolo_ros (本仓库)
  └─ ...其他依赖 (见 package.xml)
```

## Launch 文件中的包引用

### Reality 启动文件
所有 `launch/reality/*.py` 文件中会看到：

```python
bringup_dir = get_package_share_directory("pb2025_nav_bringup")
```

**说明**: 这是因为实机启动文件源自 `pb2025_nav_bringup` 包，包含了该包的启动逻辑。

### Simulation 启动文件
所有 `launch/simulation/*.py` 文件中会看到：

```python
bringup_dir = get_package_share_directory("pb2025_nav_bringup")
decision_simple_dir = get_package_share_directory("decision_simple")
```

**说明**: 仿真文件依赖导航包以及决策层包。

## 使用 bringup_main.py 的优势

`bringup_main.py` 作为新的主启动文件，具有以下优势：

1. **独立于外部包名**: 主启动文件管理环境选择逻辑
2. **清晰的抽象**: 用户只需关注 `use_sim` 参数，无需知道底层包名
3. **迁移友好**: 即使底层包改名，主启动可保持不变

## 未来改进方向

### 选项1: 完全本地化（推荐长期）
将 `pb2025_nav_bringup` 的启动逻辑完全集成到本包，移除外部依赖。

**优势**:
- 完全独立
- 版本控制更清晰
- 更易定制

**成本**:
- 需要复制和维护启动代码
- 需要管理配置文件

### 选项2: 创建适配层
创建 `adapter` 子目录，专门处理与 `pb2025_nav_bringup` 的兼容性。

### 选项3: 保持现状（当前）
继续使用现有的启动文件，通过 `bringup_main.py` 统一接口。

**优势**:
- 最少改动
- 继续获得上游包的更新

**劣势**:
- 依赖外部包名称

## 常见问题

### Q: 我的系统中没有 pb2025_nav_bringup 包怎么办？

**A**: 需要编译和安装该包：

```bash
# 如果在同一仓库中
colcon build --packages-select pb2025_nav_bringup

# 如果在不同仓库
git clone <repo>
cd <repo>
colcon build --packages-select pb2025_nav_bringup
```

### Q: 可以直接修改这些启动文件中的包名吗？

**A**: 可以，但需要注意：

1. 确保对应文件和配置在 `bringup` 包中存在
2. 测试所有启动文件确保能正常工作
3. 更新文档说明

### Q: 如何在 bringup_main.py 中隐藏这个依赖？

**A**: 已经做到了。`bringup_main.py` 是独立的，它包含了完整的启动逻辑封装。

## 构建和部署清单

- [x] bringup 包代码检查
- [x] pb2025_nav_bringup 包依赖说明
- [x] launch/reality 和 launch/simulation 组织
- [x] bringup_main.py 独立入口
- [ ] (可选) 完全本地化启动文件
- [ ] (可选) 创建适配层处理兼容性

---

**重要**: 在任何生产部署前，请确认所有依赖包都已正确安装和编译。

**更新时间**: 2026-04-28
