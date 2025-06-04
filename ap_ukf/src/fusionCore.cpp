#include "ap_ukf/node/fusionCore.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>


double normalize_angle(double angle)
{
  const double result = fmod(angle + M_PI, 2.0*M_PI);
  if(result <= 0.0) return result + M_PI;
  return result - M_PI;
}

Fusion::Fusion(double alpha, double beta, double kappa)
    : filter_(alpha, beta, kappa,
              std::bind(&Fusion::stateTransitionFunction, this,
                        std::placeholders::_1, std::placeholders::_2)),
      isInitialized_(false),
      lastMeasurementTime_(0.0) {
  // setup process and measurement models
  setupProcessModel();
  setupMeasurementModels();

  // process noise (can be reset by setProcessNoise)
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q;
  Q.setZero();
  Q.diagonal() << 0.5, 0.5, 0.5, 0.3, 0.3, 0.6, 0.25, 0.25, 0.25, 0.1,
      0.1, 0.2, 0.1, 0.1, 0.1;
  filter_.setProcessNoise(Q);
  // initial estimate covariance (can be reset by setInitialCovariance)
  initialP_.setIdentity();
  initialP_ *= 1e-9;

  reset();
}

void Fusion::reset() {
  transitionMatrix_.setIdentity();
  filter_.setCovariance(initialP_);
  filter_.setState(Eigen::Matrix<double, STATE_SIZE, 1>::Zero());

  lastMeasurementTime_ = 0.0;
  isInitialized_ = false;
}

void Fusion::predict(double deltaT) { filter_.predict(deltaT); }

void Fusion::processMeasurement(Measurement measurement) {
  double deltaT = 0.0;

  if (isInitialized_) {  // predict + update cycle
    deltaT = measurement.time - lastMeasurementTime_;

    if (deltaT > 100000.0) {
      std::cout << "Delta was very large. Suspect playing from bag file. "
                   "Setting to 0.01"
                << std::endl;
      deltaT = 0.01;
    } else if (deltaT < 0.0) {
      std::cout << "Received old reading. Skipping!" << std::endl;
      return;
    }
    // predict update cycle for the measurement
    filter_.predict(deltaT);
    if (measurement.type == MeasurementTypeDepth) {
      Eigen::Matrix<double, DEPTH_SIZE, 1> z = measurement.measurement;
      Eigen::Matrix<double, DEPTH_SIZE, DEPTH_SIZE> R = measurement.covariance;
      filter_.update(z, R, depthModel_);
    } else if (measurement.type == MeasurementTypeVel) {
      Eigen::Matrix<double, VEL_SIZE, 1> z = measurement.measurement;
      Eigen::Matrix<double, VEL_SIZE, VEL_SIZE> R = measurement.covariance;
      filter_.update(z, R, dvlModel_);
    } else if (measurement.type == MeasurementTypeImu) {
      Eigen::Matrix<double, IMU_SIZE, 1> z = measurement.measurement;
      Eigen::Matrix<double, IMU_SIZE, IMU_SIZE> R = measurement.covariance;
      filter_.update(z, R, imuModel_);
    }
    else if (measurement.type == MeasurementTypeVps) {
      Eigen::Matrix<double, VPS_SIZE, 1> z = measurement.measurement;
      Eigen::Matrix<double, VPS_SIZE, VPS_SIZE> R = measurement.covariance;
      filter_.update(z, R, gpsModel_);
    }
  } else if (measurement.type == MeasurementTypeImu) {
    // initialize filter with IMU values
    Eigen::Matrix<double, STATE_SIZE, 1> x;
    x.setZero();
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P = initialP_;
    // copy orientation
    x.segment<3>(StateRoll) = measurement.measurement.segment<3>(ImuRoll);
    P.block<3, 3>(StateRoll, StateRoll) =
        measurement.covariance.block<3, 3>(ImuRoll, ImuRoll);
    // copy angular velocity
    x.segment<3>(StateVroll) = measurement.measurement.segment<3>(ImuVroll);
    P.block<3, 3>(StateVroll, StateVroll) =
        measurement.covariance.block<3, 3>(ImuVroll, ImuVroll);
    // copy acceleration
    x.segment<3>(StateAx) = measurement.measurement.segment<3>(ImuAx);
    P.block<3, 3>(StateAx, StateAx) =
        measurement.covariance.block<3, 3>(ImuAx, ImuAx);
    filter_.setState(x);
    filter_.setCovariance(P);
    std::cout << "Initialized with IMU data" << std::endl;
    isInitialized_ = true;
  }

  if (deltaT >= 0.0) {
    lastMeasurementTime_ = measurement.time;
  }
}

Eigen::Matrix<double, STATE_SIZE, 1> Fusion::stateTransitionFunction(
    const Eigen::Matrix<double, STATE_SIZE, 1> &x, double deltaT) {
  Eigen::Matrix<double, STATE_SIZE, 1> newState;
  // check readme for full equations
  Eigen::Matrix3d R =
      (Eigen::AngleAxisd(x(StateYaw), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(x(StatePitch), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(x(StateRoll), Eigen::Vector3d::UnitX()))
          .toRotationMatrix();  // rotation matrix (ZYX)
  transitionMatrix_.block<3, 3>(StateX, StateVx) = R * deltaT;
  transitionMatrix_.block<3, 3>(StateX, StateAx) = 0.5 * R * deltaT * deltaT;
  transitionMatrix_.block<3, 3>(StateVx, StateAx) =
      Eigen::Matrix3d::Identity(3, 3) * deltaT;
  double sr = sin(x(StateRoll));
  double cr = cos(x(StateRoll));
  double tp = tan(x(StatePitch));
  double cp = cos(x(StatePitch));
  Eigen::Matrix3d T;  // angular velocity -> angle transformation matrix
  T << 1, sr * tp, cr * tp, 0, cr, -sr, 0, sr / cp, cr / cp;
  transitionMatrix_.block<3, 3>(StateRoll, StateVroll) = T * deltaT;

  newState = transitionMatrix_ * x;
  return newState;
}

void Fusion::setupProcessModel() {
  filter_.setDifferenceFunction(
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &a,
         const Eigen::Matrix<double, STATE_SIZE, 1> &b)
          -> Eigen::Matrix<double, STATE_SIZE, 1> {
        Eigen::Matrix<double, STATE_SIZE, 1> diff = a - b;
        // wrap rpy
        diff(StateRoll) = normalize_angle(diff(StateRoll));
        diff(StatePitch) = normalize_angle(diff(StatePitch));
        diff(StateYaw) = normalize_angle(diff(StateYaw));
        return diff;
      });

  filter_.setMeanFunction(
      [](const Eigen::Matrix<double, STATE_SIZE, 2 * STATE_SIZE + 1>
             &sigmaPoints,
         const Eigen::Matrix<double, 2 * STATE_SIZE + 1, 1> &weights)
          -> Eigen::Matrix<double, STATE_SIZE, 1> {
        Eigen::Matrix<double, STATE_SIZE, 1> mean;
        mean.setZero();
        // atan2(sum_sin, sum_cos) is used to wrap mean of angles
        double sum_sin[3] = {0, 0, 0};
        double sum_cos[3] = {0, 0, 0};
        for (int i = 0; i < sigmaPoints.cols(); i++) {
          for (int j = 0; j < STATE_SIZE; j++) {
            if (j == StateRoll) {
              sum_sin[0] += sin(sigmaPoints(StateRoll, i)) * weights(i);
              sum_cos[0] += cos(sigmaPoints(StateRoll, i)) * weights(i);
            } else if (j == StatePitch) {
              sum_sin[1] += sin(sigmaPoints(StatePitch, i)) * weights(i);
              sum_cos[1] += cos(sigmaPoints(StatePitch, i)) * weights(i);
            } else if (j == StateYaw) {
              sum_sin[2] += sin(sigmaPoints(StateYaw, i)) * weights(i);
              sum_cos[2] += cos(sigmaPoints(StateYaw, i)) * weights(i);
            } else {
              mean(j) += sigmaPoints(j, i) * weights(i);
            }
          }
        }
        mean(StateRoll) = atan2(sum_sin[0], sum_cos[0]);
        mean(StatePitch) = atan2(sum_sin[1], sum_cos[1]);
        mean(StateYaw) = atan2(sum_sin[2], sum_cos[2]);
        return mean;
      });
}

void Fusion::setupMeasurementModels() {

  gpsModel_.measurementFn =
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &state)
      -> Eigen::Matrix<double, VPS_SIZE, 1> {
    Eigen::Matrix<double, VPS_SIZE, 1> measurement;
    measurement(0) = state(StateX);
    measurement(1) = state(StateY);
    //ROS_INFO_STREAM("from fusion = " << state(StateZ));
    return measurement;
  };

  depthModel_.measurementFn =
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &state)
      -> Eigen::Matrix<double, DEPTH_SIZE, 1> {
    Eigen::Matrix<double, DEPTH_SIZE, 1> measurement;
    measurement(0) = state(StateZ);
    //ROS_INFO_STREAM("from fusion = " << state(StateZ));
    return measurement;
  };

  dvlModel_.measurementFn =
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &state)
      -> Eigen::Matrix<double, VEL_SIZE, 1> {
    Eigen::Matrix<double, VEL_SIZE, 1> measurement;
    measurement(0) = state(StateVx);
    measurement(1) = state(StateVy);
    measurement(2) = state(StateVz);
    return measurement;
  };

  imuModel_.measurementFn =
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &state)
      -> Eigen::Matrix<double, IMU_SIZE, 1> {
    Eigen::Matrix<double, IMU_SIZE, 1> measurement;
    measurement(ImuRoll) = state(StateRoll);
    measurement(ImuPitch) = state(StatePitch);
    measurement(ImuYaw) = state(StateYaw);
    measurement(ImuVroll) = state(StateVroll);
    measurement(ImuVpitch) = state(StateVpitch);
    measurement(ImuVyaw) = state(StateVyaw);
    measurement(ImuAx) = state(StateAx);
    measurement(ImuAy) = state(StateAy);
    measurement(ImuAz) = state(StateAz);
    return measurement;
  };
  // handle wrapping for imu's orienation values
  imuModel_.diffFn = [](const Eigen::Matrix<double, IMU_SIZE, 1> &a,
                        const Eigen::Matrix<double, IMU_SIZE, 1> &b)
      -> Eigen::Matrix<double, IMU_SIZE, 1> {
    Eigen::Matrix<double, IMU_SIZE, 1> diff = a - b;
    // wrap rpy
    diff(ImuRoll) = normalize_angle(diff(ImuRoll));
    diff(ImuPitch) = normalize_angle(diff(ImuPitch));
    diff(ImuYaw) = normalize_angle(diff(ImuYaw));
    
    return diff;
  };
  imuModel_.meanFn =
      [](const Eigen::Matrix<double, IMU_SIZE, 2 * STATE_SIZE + 1> &sigmaPoints,
         const Eigen::Matrix<double, 2 * STATE_SIZE + 1, 1> &weights)
      -> Eigen::Matrix<double, IMU_SIZE, 1> {
    Eigen::Matrix<double, IMU_SIZE, 1> mean;
    mean.setZero();
    // atan2(sum_sin, sum_cos) is used to wrap mean of angles
    double sum_sin[3] = {0, 0, 0};
    double sum_cos[3] = {0, 0, 0};
    for (int i = 0; i < sigmaPoints.cols(); i++) {
      for (int j = 0; j < IMU_SIZE; j++) {
        if (j >= 0 and j < 3) {  // index 0-2 is roll, pitch & yaw
          sum_sin[j] += sin(sigmaPoints(j, i)) * weights(i);
          sum_cos[j] += cos(sigmaPoints(j, i)) * weights(i);
        } else {
          mean(j) += sigmaPoints(j, i) * weights(i);
        }
      }
    }
    for (int i = 0; i < 3; i++) {
      mean(i) = atan2(sum_sin[i], sum_cos[i]);
    }
    return mean;
  };
}

void Fusion::setInitialCovariance(
    const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &P) {
  initialP_ = P;
  reset();
}

void Fusion::setProcessNoise(
    const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &Q) {
  filter_.setProcessNoise(Q);
  reset();
}

Eigen::Matrix<double, STATE_SIZE, 1> &Fusion::getState() {
  return filter_.getState();
}

Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &Fusion::getCovariance() {
  return filter_.getCovariance();
}

void Fusion::setState(const Eigen::Matrix<double, STATE_SIZE, 1> &x) {
  filter_.setState(x);
}

bool Fusion::isInitialized() { return isInitialized_; }

double Fusion::getLastMeasurementTime() { return lastMeasurementTime_; }