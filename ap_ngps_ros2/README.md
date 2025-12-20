# NGPS ROS2 Localization Package

NGPS localization using LightGlue for ROS2.

## Features

- Real-time camera image processing
- LightGlue-based feature matching
- Rotation detection with multiple methods
- Pose estimation and tracking
- Debug visualization
- Configurable parameters

## Dependencies

```bash
sudo apt update
sudo apt install ros-humble-rclpy ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-std-msgs ros-humble-cv-bridge ros-humble-image-transport
```

```bash
pip install -r requirements.txt
```

## Installation

1. Clone the repository:
```bash
cd /path/to/your/workspace/src
git clone <repository-url>
cd ap_ngps_ros2
```

2. Install Python dependencies:
```bash
pip install -r requirements.txt
```

3. Build the package:
```bash
cd /path/to/your/workspace
colcon build --packages-select ap_ngps_ros2
source install/setup.bash
```

## Usage

### Basic Usage

1. Launch the NGPS localization node:
```bash
ros2 launch ap_ngps_ros2 ngps_localization.launch.py
```

2. With a reference image:
```bash
ros2 launch ap_ngps_ros2 ngps_localization.launch.py reference_image_path:=/path/to/your/reference/image.tif
```

3. With custom camera topic:
```bash
ros2 launch ap_ngps_ros2 ngps_localization.launch.py camera_topic:=/your_camera/image_raw
```

### Parameters

**Launch Arguments:**
- `reference_image_path`: Path to the reference image for localization
- `camera_topic`: Camera topic to subscribe to (default: `/camera/image_raw`)
- `config_file`: Path to the YAML configuration file (default: `config/ngps_config.yaml`)

**YAML Configuration Parameters:**
- `kernel_size`: Size of the kernel for feature extraction (default: 300)
- `match_threshold`: Threshold for feature matching (default: 0.5)
- `min_matches`: Minimum number of matches required (default: 20)
- `max_rotation_change`: Maximum allowed rotation change per frame (default: 30.0 degrees)
- `rotation_std_threshold`: Maximum standard deviation for recent rotations (default: 15.0 degrees)
- `enable_rotation_smoothing`: Enable rotation smoothing (default: true)
- `enable_rotation_validation`: Enable rotation validation (default: true)
- `frame_id`: Frame ID for published messages (default: "map")

### Published Topics

- `/ngps/pose` (geometry_msgs/msg/PoseStamped): Current pose with timestamp
- `/ngps/position` (geometry_msgs/msg/PointStamped): Current position with timestamp
- `/ngps/rotation` (std_msgs/msg/Float64): Current rotation angle
- `/ngps/debug_image` (sensor_msgs/msg/Image): Debug visualization with timestamp

### Subscribed Topics

- `/camera/image_raw` (sensor_msgs/msg/Image): Input camera images

## Configuration

Edit `config/ngps_config.yaml` to modify default parameters.

## Troubleshooting

1. **CUDA not available**: The node will automatically fall back to CPU if CUDA is not available.

2. **Insufficient matches**: Adjust `match_threshold` and `min_matches` parameters.

3. **Memory issues**: The node includes automatic GPU memory cleanup, but you may need to reduce `max_keypoints` if running out of memory.

## License

MIT License
