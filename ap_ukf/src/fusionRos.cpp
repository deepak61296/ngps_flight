#include "ap_ukf/node/fusionRos.h"
#include <yaml-cpp/yaml.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

FusionRos::FusionRos(const rclcpp::NodeOptions & options)
  : Node("fusion_node", options),
    filter_(1e-3, 2, 0),
    baseLinkFrame_("eca_a9/base_footprint"),
    odomFrame_("world")
     {

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);

    odomBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    loadParams();
    filter_.setInitialCovariance(params_.initialCov);
    filter_.setProcessNoise(params_.processNoiseCov);
    filter_.reset();

    try {
      odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(
          "/topic/sensor/odom_state", 60);
      imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "/eca_a9/imu", 50,
          std::bind(&FusionRos::imuCallback, this, _1));
      vioSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odometry/vio", 50,
          std::bind(&FusionRos::vioCallback, this, _1));
      vpsSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odometry/vps", 50,
          std::bind(&FusionRos::vpsCallback, this, _1));

    } catch (std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Unable to load topic/frame. Error: %s", e.what());
    }
    last_expected = this->get_clock()->now();
    updateTimer_ = this->create_wall_timer(100ms,std::bind(&FusionRos::update, this));
  }

void FusionRos::reset() {
  clearMeasurementQueue();

  tfBuffer_->clear();

  // clear all waiting callbacks
  // ros::getGlobalCallbackQueue()->clear();
}

void FusionRos::update() {
  // warn user if update loop takes too long
  // const rclcpp::TimerBase &event;
  auto last_cycle_duration = 
      (this->now().seconds() - last_expected.seconds());

  // update the the last loop time
  last_expected = this->now();

  if (last_cycle_duration > 2. / params_.frequency) {
    // RCLCPP_WARN_STREAM(get_logger(), "Failed to meet update rate! Last cycle took "
    //                 << std::setprecision(20) << last_cycle_duration);

    RCLCPP_WARN(get_logger(), "Failed to meet update rate! Last cycle took %.2f seconds", last_cycle_duration);
  }

  // const rclcpp::Time currentTime = this->get_clock()->now();
  const rclcpp::Time currentTime = this->now();
  if (!measurementQueue_.empty()) {
    while (!measurementQueue_.empty() && rclcpp::ok()) {
      MeasurementPtr z = measurementQueue_.top();
      // if measurement's time is later than now, wait until next iteration
      if (z->time > currentTime.seconds()) {
        break;
      }
      measurementQueue_.pop();
      // predict + update loop with measurement
      filter_.processMeasurement(*(z.get()));
    }
  } else if (filter_.isInitialized()) {  // only predict if initialized
    // no measurement call filter predict
    double deltaT = currentTime.seconds() - filter_.getLastMeasurementTime();
    if (deltaT > 100000.0) {
      RCLCPP_WARN(get_logger(),
          "Delta was very large. Suspect playing from bag file. Setting to "
          "0.01");
      deltaT = 0.01;
    }
    filter_.predict(deltaT);
    // RCLCPP_WARN_THROTTLE(get_logger(), 1s, "No measurements recieved. Using prediction only.");
  }

  // publish message and frame transform
  if (filter_.isInitialized()) {
    nav_msgs::msg::Odometry filteredState = getFilteredOdomMessage();
    // broadcast odom frame
    geometry_msgs::msg::TransformStamped odomTransMsg;
//    odomTransMsg.header.stamp = filteredState.header.stamp;
    odomTransMsg.header.stamp = this->now();
    odomTransMsg.header.frame_id = filteredState.header.frame_id;
    odomTransMsg.child_frame_id = filteredState.child_frame_id;
    odomTransMsg.transform.translation.x = filteredState.pose.pose.position.x;
    odomTransMsg.transform.translation.y = filteredState.pose.pose.position.y;
    odomTransMsg.transform.translation.z = filteredState.pose.pose.position.z;
    odomTransMsg.transform.rotation = filteredState.pose.pose.orientation;

//    RCLCPP_INFO(get_logger(), "frame %s,  pos : (%.2f, %.2f, %.2f,) orient: (%.2f, %.2f, %.2f, %.2f,)", filteredState.child_frame_id.c_str(),
//                filteredState.pose.pose.position.x, filteredState.pose.pose.position.y, filteredState.pose.pose.position.z,
//                filteredState.pose.pose.orientation.x,filteredState.pose.pose.orientation.y, filteredState.pose.pose.orientation.z, filteredState.pose.pose.orientation.w);

//    odomBroadcaster_->sendTransform(odomTransMsg);
    // publish odom message
    odomPub_->publish(filteredState);
  }
}

nav_msgs::msg::Odometry FusionRos::getFilteredOdomMessage() {
  // should only be called if filter is initialized
  assert(filter_.isInitialized());

  const Eigen::VectorXd &state = filter_.getState();
  const Eigen::MatrixXd &cov = filter_.getCovariance();

  tf2::Quaternion quat;
  quat.setRPY(state(StateRoll), state(StatePitch), state(StateYaw));

  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = state(StateX);
  odom.pose.pose.position.y = state(StateY);
  odom.pose.pose.position.z = state(StateZ);
  //ROS_INFO_STREAM("from _ros = " << state(StateZ));
  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();
  odom.pose.pose.orientation.w = quat.w();
  odom.twist.twist.linear.x = state(StateVx);
  odom.twist.twist.linear.y = state(StateVy);
  odom.twist.twist.linear.z = state(StateVz);
  odom.twist.twist.angular.x = state(StateVroll);
  odom.twist.twist.angular.y = state(StateVpitch);
  odom.twist.twist.angular.z = state(StateVyaw);

  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      odom.pose.covariance[6 * i + j] = cov(i, j);
      odom.twist.covariance[6 * i + j] = cov(i + StateVx, j + StateVx);
    }
  }

  odom.header.stamp = rclcpp::Time(filter_.getLastMeasurementTime());
  odom.header.frame_id = odomFrame_;
  odom.child_frame_id = baseLinkFrame_;
  return odom;
}

void FusionRos::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  MeasurementPtr z = std::make_shared<Measurement>(IMU_SIZE);
  z->type = MeasurementTypeImu;
  auto& stamp = msg->header.stamp;
  rclcpp::Time time(stamp.sec, stamp.nanosec);
  z->time = time.seconds();
  z->covariance.setZero();
  tf2::Transform targetFrameTrans = getTransformFrame(msg->header);

  // orientation
  // note: IMU should be mounted such that RPY is in NED coord frame
  tf2::Quaternion q;
  tf2::fromMsg(msg->orientation, q);
  tf2::Matrix3x3 orientation(q);
  double roll, pitch, yaw;
  orientation.getRPY(roll, pitch, yaw);
  z->measurement(ImuRoll) = roll;
  z->measurement(ImuPitch) = pitch;
  z->measurement(ImuYaw) = yaw;
  z->covariance.block<3, 3>(ImuRoll, ImuRoll) =
      Eigen::Vector3d(msg->orientation_covariance[0],
                      msg->orientation_covariance[4],
                      msg->orientation_covariance[8])
          .asDiagonal();

  // angular velocity
  tf2::Vector3 angularVelocity(msg->angular_velocity.x, msg->angular_velocity.y,
                               msg->angular_velocity.z);
  angularVelocity = targetFrameTrans.getBasis() * angularVelocity;
  z->measurement(ImuVroll) = angularVelocity.x();
  z->measurement(ImuVpitch) = angularVelocity.y();
  z->measurement(ImuVyaw) = angularVelocity.z();
  // rotate covariance matrix to base_link
  Eigen::MatrixXd covarianceRotated(3, 3);
  rotateCovariance(&(msg->angular_velocity_covariance[0]),
                   targetFrameTrans.getRotation(), covarianceRotated);
  z->covariance.block<3, 3>(ImuVroll, ImuVroll) = covarianceRotated;

  // linear acceleration
  tf2::Vector3 linearAcceleration(msg->linear_acceleration.x,
                                  msg->linear_acceleration.y,
                                  msg->linear_acceleration.z);
  linearAcceleration = targetFrameTrans.getBasis() * linearAcceleration;
  z->measurement(ImuAx) = linearAcceleration.x();
  z->measurement(ImuAy) = linearAcceleration.y();
  z->measurement(ImuAz) = linearAcceleration.z();
  // rotate covariance matrix to base_link
  rotateCovariance(&(msg->linear_acceleration_covariance[0]),
                   targetFrameTrans.getRotation(), covarianceRotated);
  z->covariance.block<3, 3>(ImuAx, ImuAx) = covarianceRotated;

  measurementQueue_.push(z);
}

void FusionRos::vioCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  MeasurementPtr z = std::make_shared<Measurement>(VIO_SIZE);
  z->type = MeasurementTypeVio;
  auto& stamp = msg->header.stamp;
  rclcpp::Time time(stamp.sec, stamp.nanosec);
  z->time = time.seconds();
  tf2::Transform targetFrameTrans = getTransformFrame(msg->header);

  // Position
  tf2::Vector3 position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  position = targetFrameTrans.getBasis() * position;
  z->measurement(VioX) = position.x();
  z->measurement(VioY) = position.y();
  z->measurement(VioZ) = position.z();

  // Velocity
  tf2::Vector3 velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  velocity = targetFrameTrans.getBasis() * velocity;
  // account for linear velocity as a result of sensor offset and rotational velocity
  const Eigen::VectorXd &state = filter_.getState();
  tf2::Vector3 angVel(state(StateVroll), state(StateVpitch), state(StateVyaw));
  velocity += targetFrameTrans.getOrigin().cross(angVel);
  z->measurement(VioVx) = velocity.x();
  z->measurement(VioVy) = velocity.y();
  z->measurement(VioVz) = velocity.z();

  // Set covariance matrix (position + velocity)
  z->covariance.setZero();
  // Position covariance
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      z->covariance(i, j) = msg->pose.covariance[6 * i + j];
    }
  }
  // Velocity covariance
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      z->covariance(i + 3, j + 3) = msg->twist.covariance[6 * i + j];
    }
  }

  measurementQueue_.push(z);
}

void FusionRos::vpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  MeasurementPtr z = std::make_shared<Measurement>(VPS_SIZE);
  z->type = MeasurementTypeVps;

  auto& stamp = msg->header.stamp;
  rclcpp::Time time(stamp.sec, stamp.nanosec);
  z->time = time.seconds();
  //z->time = msg->header.stamp.sec();

  z->measurement(0) = msg->pose.pose.position.x;
  z->measurement(1) = msg->pose.pose.position.y;

  z->covariance(0, 0) = msg->pose.covariance[0];
  measurementQueue_.push(z);

}

void FusionRos::loadParams() {

  params_.frequency = get_parameter("frequency").as_double();
  std::vector<double> a = get_parameter("initial_estimate_covariance").as_double_array();
  std::vector<double> b = get_parameter("process_noise_covariance").as_double_array();
  params_.initialCov =  loadMatrixFromParams(a);
  params_.processNoiseCov = loadMatrixFromParams(b);
}


Eigen::MatrixXd FusionRos::loadMatrixFromParams(std::vector<double> data)
{
    Eigen::MatrixXd eMatrix(15, 15);
    int count = 0;
    for (int i=0; i<15; i++){
      for (int j=0; j<15; j++){
        eMatrix(i,j) = data[count];
        count++;
      }
    }

    return eMatrix;
}
// void FusionRos::loadMatrixFromParams(Eigen::MatrixXd &mat,
//                                            const std::string &key) {
//   size_t size = mat.rows();
//   mat.setZero();
//   XmlRpc::XmlRpcValue param;
//   Eigen::MatrixXd param(15,15);

//   try {
//     this->get_parameter(key, param);
//     for (size_t i = 0; i < size; i++) {
//       for (size_t j = 0; j < size; j++) {
//         // needed if all points don't have decimal points
//         std::ostringstream os;
//         os << param[size * i + j];
//         std::istringstream is(os.str());
//         is >> mat(i, j);
//       }
//     }
//   } catch (...) {
//     RCLCPP_ERROR(get_logger(), "Error loading initial_estimate_covariance from param file");
//   }
// }

tf2::Transform FusionRos::getTransformFrame(
    const std_msgs::msg::Header &header) {
  tf2::Transform targetFrameTrans;
  try {
    tf2::fromMsg(tfBuffer_->lookupTransform(baseLinkFrame_, header.frame_id,
                                      rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)).transform,
                 targetFrameTrans);
  } catch (tf2::TransformException &ex) {
    // RCLCPP_WARN_STREAM_THROTTLE(get_logger(), this->get_clock(), 2.0, "Could not obtain transform from "<< header.frame_id << " to "<< baseLinkFrame_<< ". Error: " << ex.what());
    RCLCPP_ERROR(get_logger(), "Could not obtain transform from");
  }
  return targetFrameTrans;
}

void FusionRos::clearMeasurementQueue() {
  while (!measurementQueue_.empty() && rclcpp::ok()) {
    measurementQueue_.pop();
  }
}

void FusionRos::rotateCovariance(const double *covariance,
                                       const tf2::Quaternion &q,
                                       Eigen::MatrixXd &rotated) {
  // create Eigen matrix with rotation q
  tf2::Matrix3x3 tfRot(q);
  Eigen::MatrixXd rot(3, 3);
  for (size_t i = 0; i < 3; ++i) {
    rot(i, 0) = tfRot.getRow(i).getX();
    rot(i, 1) = tfRot.getRow(i).getY();
    rot(i, 2) = tfRot.getRow(i).getZ();
  }
  // copy covariance to rotated
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      rotated(i, j) = covariance[3 * i + j];
    }
  }
  rotated = rot * rotated.eval() * rot.transpose();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  options.automatically_declare_parameters_from_overrides(true);
  auto nh = std::make_shared<FusionRos>(options);

  // FusionRos fusionNode;

  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}