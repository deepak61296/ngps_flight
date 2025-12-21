# NGPS ROS2 Localization Package

NGPS localization using LightGlue for ROS2.

## Features

- Real-time camera image processing
- LightGlue-based feature matching
- Rotation detection with multiple methods
- Pose estimation and tracking
- Global coordinate extraction (WGS84 and ECEF)
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
cd /path/to/workspace/src
git clone <repository-url>
cd ap_ngps_ros2
```

2. Install Python dependencies:
```bash
pip install -r requirements.txt
```

3. Build the package:
```bash
cd /path/to/workspace
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
ros2 launch ap_ngps_ros2 ngps_localization.launch.py reference_image_path:=/path/to/reference/image.tif
```

3. With custom camera topic:
```bash
ros2 launch ap_ngps_ros2 ngps_localization.launch.py camera_topic:=/camera/image_raw
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

**Georeferencing Parameters (for global coordinates):**
- `reference_min_lon`: Minimum longitude (west edge) of reference image in decimal degrees (default: 0.0)
- `reference_min_lat`: Minimum latitude (south edge) of reference image in decimal degrees (default: 0.0)
- `reference_max_lon`: Maximum longitude (east edge) of reference image in decimal degrees (default: 0.0)
- `reference_max_lat`: Maximum latitude (north edge) of reference image in decimal degrees (default: 0.0)
- `reference_altitude`: Reference altitude in meters (AGL or MSL) (default: 0.0)
- `enable_global_coordinates`: Enable publishing of global coordinates (WGS84 and ECEF) (default: false)

### Published Topics

- `/ngps/pose` (geometry_msgs/msg/PoseStamped): Current pose with timestamp (local coordinates)
- `/ngps/position` (geometry_msgs/msg/PointStamped): Current position with timestamp (local coordinates)
- `/ngps/rotation` (std_msgs/msg/Float64): Current rotation angle
- `/ngps/debug_image` (sensor_msgs/msg/Image): Debug visualization with timestamp
- `/ngps/global_position` (sensor_msgs/msg/NavSatFix): Global position in WGS84 coordinates (lat/lon/alt) - *only published if `enable_global_coordinates` is true*
- `/ngps/ecef_position` (geometry_msgs/msg/PointStamped): Position in ECEF (Earth-Centered, Earth-Fixed) coordinates - *only published if `enable_global_coordinates` is true*

### Subscribed Topics

- `/camera/image_raw` (sensor_msgs/msg/Image): Input camera images

## Configuration

Edit `config/ngps_config.yaml` to modify default parameters.

### Enabling Global Coordinates

To enable global coordinate extraction and publishing:

1. **Configure georeferencing parameters** in `config/ngps_config.yaml`:
   ```yaml
   reference_min_lon: -122.4194  # West edge longitude
   reference_min_lat: 37.7749     # South edge latitude
   reference_max_lon: -122.4094   # East edge longitude
   reference_max_lat: 37.7849      # North edge latitude
   reference_altitude: 100.0       # Altitude in meters
   enable_global_coordinates: true
   ```

2. **Determine reference image bounding box**:
   - If reference image is a GeoTIFF, can extract the bounding box using tools like `gdalinfo`
   - For satellite imagery, use the coordinates from the imagery provider
   - The bounding box should be in WGS84 (EPSG:4326) decimal degrees

3. **Global coordinates will be published** to:
   - `/ngps/global_position` (NavSatFix): WGS84 latitude, longitude, altitude
   - `/ngps/ecef_position` (PointStamped): ECEF coordinates in meters

## Troubleshooting

1. **CUDA not available**: The node will automatically fall back to CPU if CUDA is not available.

2. **Insufficient matches**: Adjust `match_threshold` and `min_matches` parameters.

3. **Memory issues**: The node includes automatic GPU memory cleanup, but may need to reduce `max_keypoints` if running out of memory.

## License

MIT License
