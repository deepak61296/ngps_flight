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
#include <sensor_msgs/msg/fluid_pressure.hpp>
// #include "sensor_msgs/msg/nav_sat_fix.h"
// #include "sensor_msgs/msg/nav_sat_fix.h"


#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>


#include <ap_ukf/node/fusionCore.h>

using MeasurementQueue =
    std::priority_queue<MeasurementPtr, std::vector<MeasurementPtr>,
                        Measurement>;

/*
 * ROS wrapper for Fusion class.
 * subscribes to all sensor inputs
 * publishes filtered output message
 */
class FusionRos: public rclcpp::Node {
 public:
  // constructor
  // explicit FusionRos(const ros::NodeHandle& nh,
  //                          const ros::NodeHandle& private_nh);

  explicit FusionRos(const rclcpp::NodeOptions & options);

  // destructor - closes all subscriber connections and clears the message
  // filters
  ~FusionRos() = default;

  // resets filter to initial state
  void reset();

 protected:
  // periodically called by ros::Timer    const rclcpp::TimerBase& event
  void update();

  // callback method for receiving IMU messages
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // callback method for receiving DVL messages
  void dvlCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  void gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // callback method for recieving depth messages
  void depthCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);

  // returns the ekf output as ros message for publishing
  nav_msgs::msg::Odometry getFilteredOdomMessage();

  rclcpp::Time last_expected;

  // gets transform frame from message from to baselink
  tf2::Transform getTransformFrame(const std_msgs::msg::Header& header);

  // loads ukf params from file
  void loadParams();

  // loads square eigen matrix from params
  Eigen::MatrixXd loadMatrixFromParams(std::vector<double> data);

  // clears measurement queue
  void clearMeasurementQueue();

  // rotates covariance array using quaternion q
  // result copies to Eigen::Matrix rotated
  void rotateCovariance(const double* covariance, const tf2::Quaternion& q,
                        Eigen::MatrixXd& rotated);

  // ros node handles
  // ros::NodeHandle nh_;
  // ros::NodeHandle private_nh_;

  struct Params {
    // filter update rate
    double frequency;
    // ukf sigma parameters
    Eigen::MatrixXd initialCov;
    Eigen::MatrixXd processNoiseCov;

    Params()
        : initialCov(STATE_SIZE, STATE_SIZE),
          processNoiseCov(STATE_SIZE, STATE_SIZE) {}
  } params_;

  // fusion ukf filter
  Fusion filter_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
  rclcpp::TimerBase::SharedPtr updateTimer_;
  // subscribers for sensors
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvlSub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr depthSub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gpsSub_;

  // frames ids
  std::string baseLinkFrame_;
  std::string odomFrame_;

  

  // tf buffer for managing coord frames
  // tf2_ros::Buffer tfBuffer_;
  // tf2_ros::TransformListener tfListener_;
  // tf2_ros::TransformBroadcaster odomBroadcaster_;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;

  //! @brief Transform listener for receiving transforms
  //!
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;

  //! @brief broadcaster of worldTransform tfs
  //!
  std::shared_ptr<tf2_ros::TransformBroadcaster> odomBroadcaster_;

  // queue measurements until update method is trigger
  MeasurementQueue measurementQueue_;
};