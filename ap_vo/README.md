# ap_vo

Visual Odometry node for NGPS flight system.

## Features

- SIFT-based feature detection and matching
- PnP pose estimation
- Scale recovery using distance to ground
- Publishes relative pose in odom frame

## Dependencies

```bash
sudo apt install ros-humble-rclpy ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
```

## Usage

```bash
ros2 launch ap_vo vo.launch.py
```

## Published Topics

- `/vo/pose` (geometry_msgs/msg/PoseWithCovarianceStamped): Relative pose in odom frame

## Subscribed Topics

- `/camera/image_raw` (sensor_msgs/msg/Image): Camera images
- `/camera/camera_info` (sensor_msgs/msg/CameraInfo): Camera intrinsics

## Parameters

- `confidence_threshold`: Ratio test threshold for matches (default: 0.7)
- `min_matches`: Minimum matches required (default: 30)
- `camera_topic`: Camera image topic (default: `/camera/image_raw`)
- `camera_info_topic`: Camera info topic (default: `/camera/camera_info`)
- `output_frame`: Output frame ID (default: `odom`)
