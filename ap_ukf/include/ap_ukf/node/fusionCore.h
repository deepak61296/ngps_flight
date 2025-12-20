#pragma once

// #include <au_core/math_util.h>
#include <ap_ukf/core/stateTypes.h>
#include <ap_ukf/core/stateFuser.h>
#include <iostream>
#include <memory>

/*
 * fusion filter based on UKF
 */
class Fusion {
 public:
  /*
   * initializes the filter
   * @param alpha: spread of sigma points (1 < alpha < 1e-4)
   * @param beta: prior knowledge of distribution (Gaussian = 2)
   * @param kappa: secondary scaling parameter
   */
  Fusion(double alpha, double beta, double kappa);

  ~Fusion() = default;

  /*
   * resets filter state
   * call after setting initial covariance or process noise
   */
  void reset();

  /*
   * calls predict and update for a sensor measurement
   * if filter is not initialized, initializes with IMU data
   */
  void processMeasurement(Measurement measurement);

  /*
   * runs the predict step only
   * @param: deltaT: prediction time in seconds
   */
  void predict(double deltaT);

  /*
   * set initial covariance of filter
   * @param P: initial covariance
   */
  void setInitialCovariance(
      const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>& P);

  /*
   * set process noise covariance
   * @param Q: noise covariance
   */
  void setProcessNoise(const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>& Q);

  /*
   * set state of filter
   * @param x: state
   */
  void setState(const Eigen::Matrix<double, STATE_SIZE, 1>& x);

  /*
   * get current filter estimate
   * @return: filter state estimate
   */
  Eigen::Matrix<double, STATE_SIZE, 1>& getState();

  /*
   * get filter covariance
   * @return covariance
   */
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>& getCovariance();

  /*
   * get initialization state
   * @return: true if initialized
   */
  bool isInitialized();

  /*
   * get time for last measurement
   * @param: time in seconds
   */
  double getLastMeasurementTime();

 protected:
  /*
   * state transition function (process model)
   * 3d kinematics model for the robot
   */
  Eigen::Matrix<double, STATE_SIZE, 1> stateTransitionFunction(
      const Eigen::Matrix<double, STATE_SIZE, 1>& x, double deltaT);

  /*
   * custom difference and mean function to handle angular values
   */
  void setupProcessModel();

  /*
   * setup measurement models for imu, vio and vps
   */
  void setupMeasurementModels();

  // unscented kalman filter
  Ukf<STATE_SIZE> filter_;

  // filter initialized flag
  bool isInitialized_;
  // timestamp of most recent measurement
  double lastMeasurementTime_;

  // state transition matrix
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> transitionMatrix_;
  // initial covariance
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> initialP_;

  // measurement models
  MeasurementModel<VIO_SIZE, STATE_SIZE> vioModel_;
  MeasurementModel<IMU_SIZE, STATE_SIZE> imuModel_;
  MeasurementModel<VPS_SIZE, STATE_SIZE> vpsModel_;
};