PX4 + Ignition SITL launcher

Usage:

- Place an SDF model for the vehicle at `src/simulator/resource/models/iris/model.sdf` (or change `model_sdf` launch arg).
- Launch with:

```bash
ros2 launch simulator px4_ignition_sitl.launch.py
```

Arguments (optional):

- `px4_dir`: PX4 source directory (default `~/PX4-Autopilot`).
- `world_file`: Ignition world to load (default `src/simulator/resource/worlds/gimbal_sim.sdf`).
- `model_sdf`: SDF file to spawn (default `src/simulator/resource/models/iris/model.sdf`).

Notes:

- This launcher runs external commands (`make px4_sitl_default`, `ign gazebo`, `ros2 run ros_ign_gazebo create`, and `px4_ros_com`). Ensure those tools are installed and on PATH.
- If you prefer `mavros` instead of `px4_ros_com`, replace the last ExecuteProcess command in the launch file accordingly.
