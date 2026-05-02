# IMMKF Predictor - Quick Reference

## Basic Usage

```bash
# Default launch (standard configuration)
ros2 launch immkf_predictor immkf_predictor.launch.py

# Conservative tracking (longer track lifetime, all classes)
ros2 launch immkf_predictor immkf_predictor.launch.py \
  config:=$(find immkf_predictor)/config/immkf_predictor_conservative.yaml

# Aggressive tracking (short timeout, persons only)
ros2 launch immkf_predictor immkf_predictor.launch.py \
  config:=$(find immkf_predictor)/config/immkf_predictor_aggressive.yaml
```

## Topic Mapping

```
INPUT:  /tracks_3d (yolo_msgs/DetectionArray)
OUTPUT: /immkf/tracks/{track_id}/path (nav_msgs/Path)
        /immkf/poses/{track_id}/current (geometry_msgs/PoseStamped)
        /immkf/tracks/all_tracks (nav_msgs/Path) [if publish_aggregated=true]
```

## Configuration Override

```bash
# Override specific parameters
ros2 launch immkf_predictor immkf_predictor.launch.py \
  topics_input:=/my_detections \
  track_timeout:=3.0 \
  max_tracks:=50 \
  min_confidence:=0.6
```

## Performance Tuning

| Scenario | Recommendation |
|----------|-----------------|
| Crowded scene (>30 objects) | ↓ max_tracks, ↑ min_confidence |
| Occluded environment | ↑ track_timeout, ↑ initial_covariance |
| High-speed vehicles | ↓ prediction_dt, ↑ prediction_horizon |
| Pedestrians in open space | ↑ min_confidence, ↓ track_timeout |

## Class ID Reference (COCO Format)

```
0: person        5: bus          10: traffic light
1: bicycle       6: train        11: fire hydrant
2: car           7: truck        12: stop sign
3: motorcycle    8: boat         13: parking meter
4: airplane      9: bench        14: cat
```

## Debugging Commands

```bash
# Monitor published topics in real-time
ros2 topic list | grep immkf

# Check message frequency
ros2 topic hz /immkf/tracks/track_001/path

# Inspect published messages
ros2 topic echo /immkf/tracks/track_001/path

# Check node parameters
ros2 param list /immkf_predictor_node
ros2 param get /immkf_predictor_node track_timeout_seconds
```

## State Vector Guide

```
State[0] = x position (meters)
State[1] = y position (meters)
State[2] = velocity in x (m/s)
State[3] = velocity in y (m/s)
State[4] = acceleration in x (m/s²)
State[5] = acceleration in y (m/s²)

Measurement[0] = bbox center x (from detection)
Measurement[1] = bbox center y (from detection)
```

## Common Issues & Solutions

| Problem | Solution |
|---------|----------|
| Too many false tracks | ↑ min_confidence, ↓ track_timeout |
| Tracks disappear | ↓ min_confidence, ↑ track_timeout |
| Jerky predictions | ↑ measurement_noise_x/y, ↓ initial_covariance |
| Smooth but delayed | ↓ measurement_noise_x/y, ↑ initial_covariance |
| CPU overload | ↓ max_tracks, ↓ prediction_horizon |

## Integration Example

```python
# In your launch file
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='perception_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='immkf_predictor',
                    plugin='immkf_predictor::PredictorNode',
                    name='immkf_node',
                    remappings=[
                        ('/immkf_predictor_node/topics/input', '/yolo/detections'),
                    ],
                    parameters=[{
                        'track_timeout_seconds': 5.0,
                        'max_tracks': 100,
                        'min_confidence': 0.5,
                    }],
                ),
            ],
        ),
    ])
```

## Advanced Configuration

### High-precision scenario (clean environments):
```yaml
measurement_noise_x: 0.2
measurement_noise_y: 0.2
initial_covariance_diagonal: [0.5, 0.5, 0.1, 0.1, 0.1, 0.1]
min_confidence: 0.8
```

### Robust scenario (messy environments):
```yaml
measurement_noise_x: 2.0
measurement_noise_y: 2.0
initial_covariance_diagonal: [3.0, 3.0, 2.0, 2.0, 2.0, 2.0]
min_confidence: 0.3
track_timeout_seconds: 10.0
```

### Real-time prediction scenario:
```yaml
prediction_dt: 0.05
prediction_horizon: 20
publish_per_track: true
publish_aggregated: false
```

## Performance Metrics

```
# Typical compilation time: 11-12s
# Typical update latency: 2-5ms
# Memory per track: ~500 bytes
# Recommended max_tracks: 100-150

# For N tracks with H horizon steps:
# Computational cost ∝ N × H × 4 (models)
# N=50, H=10: ~2000 elementary operations per update
```

## File Structure

```
immkf_predictor/
├── README.md                          # Full documentation
├── QUICKREF.md                        # This file
├── CMakeLists.txt                     # Build configuration
├── package.xml                        # Package metadata
├── include/immkf_predictor/
│   ├── immkf.hpp                      # Core algorithm (headers)
│   └── predictor_node.hpp             # ROS node interface
├── src/
│   ├── immkf.cpp                      # Core algorithm (implementation)
│   └── predictor_node.cpp             # ROS node (implementation)
├── config/
│   ├── immkf_predictor.yaml           # Default parameters
│   ├── immkf_predictor_conservative.yaml
│   └── immkf_predictor_aggressive.yaml
└── launch/
    └── immkf_predictor.launch.py      # Launch configuration
```
