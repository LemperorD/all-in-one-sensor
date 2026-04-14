# IMMKF Trajectory Predictor

Interactive Multi-Model Kalman Filter (IMMKF) for UAV trajectory prediction in ROS2.

## Overview

This package implements an IMMKF-based trajectory prediction system that:

1. **Subscribes** to fused detection results (`/fused_detections`)
2. **Maintains** three motion models for each tracked object:
   - **Constant Velocity (CV)**: For uniform motion
   - **Constant Acceleration (CA)**: For accelerated motion
   - **Singer Model**: For maneuvering flight with decay
3. **Intelligently switches** between models based on movement patterns
4. **Publishes** predicted trajectories (`/predicted_trajectories`)

## System Architecture

```
sensor_fusion_node (lidar_detector)
        ↓
fused_detections (DetectionArray)
        ↓
immkf_predictor_node (IMMKF)
        ↓
predicted_trajectories (PredictedTrajectoryArray)
        ↓
mpc_gimbal_planner
```

## Installation

### Prerequisites
- ROS2 (tested with Humble)
- Eigen3
- yolo_msgs

### Build
```bash
cd ~/all-in-one-sensor
colcon build --packages-select immkf_predictor
source install/setup.bash
```

## Usage

### Launch the node
```bash
ros2 launch immkf_predictor immkf_predictor.launch.py
```

### With custom parameters
```bash
ros2 launch immkf_predictor immkf_predictor.launch.py \
    input_topic:=/custom_detections \
    output_topic:=/custom_predictions
```

## Running Tests

### Motion model tests
```bash
colcon test --packages-select immkf_predictor --ctest-args -V
```

### Individual test
```bash
ros2 run immkf_predictor test_motion_models
ros2 run immkf_predictor test_immkf_filter
```

## Configuration

Edit `config/immkf_default.yaml` to tune:

- **prediction.horizon_seconds**: Prediction time window (default: 0.5s)
- **models.*.process_noise_***: Model process noise (higher = more dynamic)
- **measurement.position_noise**: Detection measurement noise
- **tracking.max_track_age**: Frames before deleting tracks

Example configuration for aggressive maneuvers:
```yaml
immkf_node:
  ros__parameters:
    models:
      singer_model:
        decay_rate: 0.90  # Faster decay = more responsive to maneuvers
```

## Messages

### Input: `/fused_detections` (yolo_msgs/DetectionArray)
- Contains Detection messages with:
  - `id`: Object tracking ID
  - `bbox3d.center.position`: 3D position [x, y, z]
  - `class_name`: Object class
  - `score`: Detection confidence

### Output: `/predicted_trajectories` (immkf_predictor/PredictedTrajectoryArray)
- Array of PredictedTrajectory messages:
  - `object_id`: Tracking ID
  - `trajectory`: Array of predicted positions per 0.1s
  - `model_probabilities`: [p_cv, p_ca, p_singer]
  - `track_confidence`: Overall track reliability

## Algorithm Details

### IMMKF Cycle (per detection):

1. **Model Mixing**: Blend previous model states
2. **Prediction**: Each model predicts for time dt
3. **Update**: Incorporate measurement (detection)
4. **Model Probability Update**: Based on innovation likelihood
5. **State Fusion**: Weighted average of all model states

### Motion Models

**Constant Velocity (CV) - 6D State: [x, y, z, vx, vy, vz]**
```
x(k+1) = x(k) + vx(k)*dt
v(k+1) = v(k)  (constant)
```

**Constant Acceleration (CA) - 9D State: [x, y, z, vx, vy, vz, ax, ay, az]**
```
x(k+1) = x(k) + v(k)*dt + 0.5*a(k)*dt²
v(k+1) = v(k) + a(k)*dt
a(k+1) = a(k)  (constant)
```

**Singer Model - 9D State: [x, y, z, vx, vy, vz, ax, ay, az]**
```
Acceleration decays: a(t) = a(0)*exp(-α*t)
More realistic for UAVs with adaptive maneuvers
```

## Example: Manual Testing

### Test Case 1: Constant Velocity
```python
# Send detections moving at constant velocity
for i in range(10):
    pos = [i*0.1, 0, 0]  # Moving in X direction
    send_detection(object_id="uav1", position=pos, time=i*0.1)
```

Expected: CV model probability → 0.8+

### Test Case 2: Acceleration
```python
# Send detections with accelerating motion
for i in range(10):
    pos = [0.05*i*i, 0, 0]  # Quadratic trajectory
    send_detection(object_id="uav1", position=pos, time=i*0.1)
```

Expected: CA model probability rises as acceleration is detected

### Test Case 3: Maneuver
```python
# Send detections with declining acceleration
for i in range(10):
    accel = 2.0 * (0.95**i)  # Exponentially decaying
    vel = sum([2.0 * (0.95**j) for j in range(i)])
    pos = [sum([vel*0.1 for _ in range(i)]), 0, 0]
    send_detection(object_id="uav1", position=pos, time=i*0.1)
```

Expected: Singer model probability rises as maneuvering detected

## Performance Metrics

Target KPIs:
- **Prediction error (0.5s horizon)**: < 10cm RMS
- **Model switching latency**: < 200ms
- **Node processing latency**: < 100ms
- **Max tracks**: 100+ simultaneous objects

## Troubleshooting

### High prediction errors
- Increase `measurement.position_noise` if detections are noisy
- Tune model process noise parameters

### Slow model switching
- Increase `decay_rate` for Singer model (makes it more responsive)
- Decrease mode transition matrix diagonal values

### Node crashes
- Check input topic connectivity: `ros2 topic echo /fused_detections`
- Verify message types: `ros2 message show immkf_predictor/msg/PredictedTrajectory`

## References

- Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation*
- Singer, R. A. (1970). Estimating optimal tracking filter performance for manned maneuvering targets
- IMM filter theory: https://en.wikipedia.org/wiki/Interacting_multiple_model

## License

Apache License 2.0

## Author

UAV Tracking Team
