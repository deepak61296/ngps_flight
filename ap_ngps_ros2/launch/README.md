# Unified Localization System

Launch files for running the complete localization system.

## Quick Start

```bash
ros2 launch ap_ngps_ros2 unified_localization_simple.launch.py
```

```bash
ros2 launch ap_ngps_ros2 unified_localization_simple.launch.py reference_image_path:=/path/to/reference.jpg
```

```bash
ros2 launch ap_ngps_ros2 unified_localization_simple.launch.py camera_topic:=/my_camera/image_raw
```

## System Architecture

```
Camera + IMU → VIPS → VIO Odometry → UKF → Fused Odometry
     ↓              ↓
   NGPS → Absolute Position → UKF
```

## Topic Flow

### Input Topics
- `/camera/image_raw` - Camera images
- `/imu/data` - IMU data

### Output Topics
- `/fused/odometry` - Final fused odometry
- `/odometry/vio` - VIO odometry from VIPS
- `/odometry/vps` - Absolute position from NGPS
- `/vips/path` - VIPS trajectory path
- `/vips/pose` - VIPS pose estimates

## Configuration Files

- **NGPS**: `ap_ngps_ros2/config/ngps_config.yaml`
- **VIPS**: `vins/config/high_alt/high_alt_mono_imu_config.yaml`
- **UKF**: `ap_ukf/params/estimator_config.yaml`

## Monitoring

```bash
ros2 node list | grep -E '(ngps_localization_node|vips_node|fusion_ros)'
```

```bash
ros2 topic hz /fused/odometry /odometry/vio /odometry/vps /imu/data
```

```bash
ros2 topic echo /fused/odometry
```

## Troubleshooting

1. **No camera data**
   ```bash
   ros2 topic list | grep camera
   ```

2. **No IMU data**
   ```bash
   ros2 topic list | grep imu
   ```

3. **VIPS not publishing VIO**
   ```bash
   ros2 topic echo /odometry/vio
   ```

4. **NGPS not publishing position**
   ```bash
   ros2 topic echo /odometry/vps
   ```

5. **UKF not publishing fused odometry**
   ```bash
   ros2 topic echo /fused/odometry
   ```

## Launch Arguments

- `reference_image_path` - Path to reference image for NGPS
- `camera_topic` - Camera topic to subscribe to
- `ngps_config_file` - NGPS configuration file path
- `vips_config_file` - VIPS configuration file path  
- `ukf_config_file` - UKF configuration file path
- `use_sim_time` - Use simulation time

## Dependencies

```bash
colcon build --packages-select ap_ngps_ros2 ap_ukf vins
```
