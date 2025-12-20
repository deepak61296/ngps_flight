#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>
#include <queue>

// #include <au_core/loader_util.h>

#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <yaml-cpp/yaml.h>

// #include <au_core/Depth.h>
// #include <au_core/Dvl.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
// #include <sensor_msgs/msg/fluid_pressure.hpp>
// #include "sensor_msgs/msg/nav_sat_fix.h"
// #include "sensor_msgs/msg/nav_sat_fix.h"


// #include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>


#include <ap_ukf/node/fusionCore.h>

using MeasurementQueue =
    std::priority_queue<MeasurementPtr, std::vector<MeasurementPtr>,
                        Measurement>;

class FusionRos: public rclcpp::Node {
 public:
  explicit FusionRos(const rclcpp::NodeOptions & options);
  ~FusionRos() = default;
  void reset();

 protected:
  void update();
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void vioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void vpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  nav_msgs::msg::Odometry getFilteredOdomMessage();

  rclcpp::Time last_expected;

  tf2::Transform getTransformFrame(const std_msgs::msg::Header& header);
  void loadParams();
  Eigen::MatrixXd loadMatrixFromParams(std::vector<double> data);
  void clearMeasurementQueue();
  void rotateCovariance(const double* covariance, const tf2::Quaternion& q,
                        Eigen::MatrixXd& rotated);

  // ros node handles
  // ros::NodeHandle nh_;
  // ros::NodeHandle private_nh_;

  struct Params {
    double frequency;
    Eigen::MatrixXd initialCov;
    Eigen::MatrixXd processNoiseCov;

    Params()
        : initialCov(STATE_SIZE, STATE_SIZE),
          processNoiseCov(STATE_SIZE, STATE_SIZE) {}
  } params_;

  Fusion filter_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
  rclcpp::TimerBase::SharedPtr updateTimer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vioSub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vpsSub_;
  std::string baseLinkFrame_;
  std::string odomFrame_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odomBroadcaster_;
  MeasurementQueue measurementQueue_;
};