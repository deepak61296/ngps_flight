# ap_ukf

Robot state estimation using Unscented Kalman Filter (UKF)
This package fuses multiple sensor inputs to estimate the robot's pose and velocity.

Core Logic

All sensor measurements are transformed to the robot's base_link frame and added to a temporal priority queue. The UKF is triggered at a fixed frequency, processing all queued measurements up to the current time. The estimated state and transform (odom → base_link) are then published.

At startup, the filter initializes upon receiving the first IMU message.

## Sensor Configuration

The filter currently fuses three sensor types:
- **IMU**: Provides orientation (roll, pitch, yaw), angular velocity, and linear acceleration
- **VIO**: Provides position (x, y, z) and linear velocity (vx, vy, vz) at high frequency (~10Hz)
- **VPS**: Provides absolute position (x, y) at low frequency (1-2Hz) for drift correction

## State Definition

The filter tracks a 15D state vector:

    Position: x, y, z

    Orientation: roll (φ), pitch (θ), yaw (ψ)

    Linear velocity: ẋ, ẏ, ż

    Angular velocity: φ̇, θ̇, ψ̇

    Linear acceleration: ẍ, ÿ, z̈

    Pose is in world frame

    Velocities are in robot's base_link frame

Predict Step

Uses a 3D kinematics model assuming:

    Constant linear acceleration

    Constant angular velocity

Orientation is propagated using standard Euler angle dynamics. Linear position and velocity are predicted using rotation matrices and acceleration in the body frame. Note: orientation update is undefined at θ = ±90° due to singularities.

### Reference

https://github.com/rummanwaqar
