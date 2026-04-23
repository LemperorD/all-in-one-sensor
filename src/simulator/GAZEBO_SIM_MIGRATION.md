# Gazebo Sim Migration Guide

## Overview
The simulator has been migrated from **Gazebo Classic** to **Gazebo Sim (formerly Ignition Gazebo)**.

### Key Changes

#### 1. **Physics Engine**
- **Old**: ODE physics engine
- **New**: DART physics engine (default in Gazebo Sim)

#### 2. **File Format**
- **Old**: `.world` files (Gazebo Classic format)
- **New**: `.sdf` files (SDF 1.9 format for Gazebo Sim)

#### 3. **Material System**
- **Old**: Material scripts with external file references
- **New**: Inline material definitions with ambient/diffuse/specular

#### 4. **Dependencies**
```bash
# Required packages
ros-${ROS_DISTRO}-ros-gz-sim        # Gazebo Sim integration
ros-${ROS_DISTRO}-ros-gz-bridge     # ROS 2 <-> Gazebo Sim bridge
```

#### 5. **Environment Variables**
- **Old**: `IGN_GAZEBO_RESOURCE_PATH`, `IGN_FILE_PATH`
- **New**: `GZ_SIM_RESOURCE_PATH` (unified variable)

### File Structure

```
simulator/
├── worlds/
│   └── gimbal_sim.sdf           # Gazebo Sim world (SDF 1.9)
├── ign/
│   └── gui.config               # GUI configuration
├── launch/
│   ├── gazebo_sim.launch.py     # Gazebo Sim launcher
│   └── full_system.launch.py    # Complete system with all nodes
├── env-hooks/
│   └── gazebo_sim.dsv.in        # Environment setup (for colcon)
└── src/
    ├── gimbal_controller_node.cpp
    ├── target_tracker_node.cpp
    └── gazebo_bridge_node.cpp
```

### Quick Start

#### Option 1: Full System Launch
```bash
ros2 launch simulator full_system.launch.py
```

#### Option 2: Gazebo Sim Only (Headless available)
```bash
# With GUI
ros2 launch simulator gazebo_sim.launch.py

# Headless (no GUI)
GZ_GUI_PLUGIN_PATH='' ros2 launch simulator gazebo_sim.launch.py
```

#### Option 3: Custom World File
```bash
ros2 launch simulator gazebo_sim.launch.py world_sdf_path:=/path/to/custom.sdf
```

### Features

1. **SDF 1.9 Support**
   - Updater physics plugins for DART
   - Scene manager integration
   - User commands system

2. **Improved Material System**
   - Direct RGBA material definition in SDF
   - No external material file dependencies
   - Better cross-platform compatibility

3. **ROS 2 Bridge**
   - Automatic clock synchronization
   - Multi-topic bridging support
   - Parameter-based configuration

4. **Ground Truth Simulation**
   - Static gimbal platform model
   - Optional camera and sensor markers
   - Configurable light sources

### SDF World Structure

The `gimbal_sim.sdf` includes:

```xml
<world name="gimbal_sim">
  <!-- Physics plugins -->
  <plugin name="gz::sim::systems::PhysicsSystem" ... />
  
  <!-- World configuration -->
  <physics name="default_physics" type="dart">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <gravity>0 0 -9.81</gravity>
  </physics>
  
  <!-- Scene setup -->
  <scene>
    <ambient>0.4 0.4 0.4 1</ambient>
    <background>0.7 0.7 0.7 1</background>
    <shadows>true</shadows>
  </scene>
  
  <!-- Lighting -->
  <light name="sun" type="directional">...</light>
  
  <!-- Models -->
  <model name="ground_plane">...</model>
  <model name="gimbal_platform">...</model>
</world>
```

### Configuration

#### GUI Configuration (`ign/gui.config`)
- Window layout and plugins
- Camera settings
- Visualization options

#### Environment Setup (`env-hooks/gazebo_sim.dsv.in`)
```bash
prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share/@PROJECT_NAME@/models
prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share/@PROJECT_NAME@/worlds
prepend-non-duplicate;SDF_PATH;share/@PROJECT_NAME@/models
```

### Troubleshooting

#### Gazebo Sim Not Found
```bash
sudo apt install ros-${ROS_DISTRO}-ros-gz-sim
```

#### GUI Not Showing
```bash
# Check graphics
export MESA_GL_VERSION_OVERRIDE=3.3
ros2 launch simulator gazebo_sim.launch.py
```

#### Bridge Connection Issues
```bash
# Verify bridge is running
ros2 node list | grep parameter_bridge

# Check published topics
ros2 topic list
```

### Performance Tips

1. **Headless Mode** for faster simulation
   ```bash
   GZ_GUI_PLUGIN_PATH='' ros2 launch simulator gazebo_sim.launch.py
   ```

2. **Physics Update Rate** can be adjusted in SDF
   ```xml
   <real_time_update_rate>1000</real_time_update_rate>
   ```

3. **Verbosity Levels**
   ```bash
   # Add to gz_args in launch file
   -v 0  # Errors only
   -v 1  # Warnings
   -v 2  # Info
   -v 3  # Debug
   ```

### References

- [Gazebo Sim Documentation](https://gazebosim.org)
- [ROS 2 Gazebo Bridge](https://github.com/gazebosim/ros_gz)
- [SDF Specification](https://sdformat.org)
