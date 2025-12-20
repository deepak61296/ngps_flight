#include <ap_ukf/node/fusionCore.h>
#include <gtest/gtest.h>

TEST(FusionTests, prediction) {
  Fusion fusion(1e-2, 2, 0);
  fusion.reset();

  Eigen::Matrix<double, STATE_SIZE, 1> x;
  x << 0, 0, 0, 0, 0, 0, 0.3, 0.2, 0.1, 0.5, 0.6, 0.7, 0.05, -0.05, 0;
  fusion.setState(x);
  fusion.predict(0.1);

  Eigen::Matrix<double, STATE_SIZE, 1> expectedX;
  expectedX << 0.3 * 0.1 + 0.05 * 0.01 * 0.5, 0.2 * 0.1 - 0.05 * 0.01 * 0.5,
      0.1 * 0.1, 0.05, 0.06, 0.07, 0.3 + 0.05 * 0.1, 0.2 - 0.05 * 0.1, 0.1, 0.5,
      0.6, 0.7, 0.05, -0.05, 0;
  Eigen::Matrix<double, STATE_SIZE, 1> expectedP;
  expectedP << 0.05, 0.05, 0.05, 0.03, 0.03, 0.06, 0.025, 0.025, 0.025, 0.01,
      0.01, 0.02, 0.01, 0.01, 0.01;

  for (int i = 0; i < STATE_SIZE; i++) {
    EXPECT_FLOAT_EQ(expectedX(i), fusion.getState()(i));
    EXPECT_FLOAT_EQ(expectedP(i), fusion.getCovariance().diagonal()(i));
  }
}

TEST(FusionTests, initialization) {
  Fusion fusion(1e-2, 2, 0);
  fusion.reset();
  EXPECT_FALSE(fusion.isInitialized());

  // VPS sensor should not trigger initialization
  Measurement vps(VPS_SIZE);
  vps.type = MeasurementTypeVps;
  vps.measurement << 1.0, 2.0;
  vps.covariance << 1e-4, 0, 0, 1e-4;
  fusion.processMeasurement(vps);
  EXPECT_FALSE(fusion.isInitialized());

  // imu sensor should trigger initialization
  Measurement imu(IMU_SIZE);
  imu.type = MeasurementTypeImu;
  imu.measurement(ImuRoll) = imu.measurement(ImuPitch) =
      imu.measurement(ImuYaw) = M_PI_4;
  imu.measurement(ImuVroll) = imu.measurement(ImuVpitch) =
      imu.measurement(ImuVyaw) = 0.2;
  imu.measurement(ImuAx) = imu.measurement(ImuAy) = imu.measurement(ImuAz) =
      0.0;
  imu.covariance.block<3, 3>(ImuRoll, ImuRoll) =
      Eigen::Matrix3d::Identity(3, 3);
  imu.covariance.block<3, 3>(ImuVroll, ImuVroll) =
      Eigen::Matrix3d::Identity(3, 3) * 0.1;
  imu.covariance.block<3, 3>(ImuAx, ImuAx) =
      Eigen::Matrix3d::Identity(3, 3) * 0.05;
  fusion.processMeasurement(imu);
  EXPECT_TRUE(fusion.isInitialized());

  Eigen::VectorXd expectedState(STATE_SIZE);
  expectedState << 0, 0, 0, M_PI_4, M_PI_4, M_PI_4, 0, 0, 0, 0.2, 0.2, 0.2, 0,
      0, 0;
  Eigen::VectorXd expectedCov(STATE_SIZE);
  expectedCov << 1e-9, 1e-9, 1e-9, 1, 1, 1, 1e-9, 1e-9, 1e-9, 0.1, 0.1, 0.1,
      0.05, 0.05, 0.05;
  for (int i = 0; i < STATE_SIZE; i++) {
    EXPECT_FLOAT_EQ(expectedState(i), fusion.getState()(i));
    EXPECT_FLOAT_EQ(expectedCov(i), fusion.getCovariance().diagonal()(i));
  }
}

TEST(FusionTests, vioMeasurement) {
  Fusion fusion(1e-2, 2, 0);
  auto cov = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity(
                 STATE_SIZE, STATE_SIZE) *
             0.1;
  fusion.setInitialCovariance(cov);
  fusion.reset();

  // initialize fusion class with IMU measurement
  Measurement imu(IMU_SIZE);
  imu.type = MeasurementTypeImu;
  imu.time = 0.0;
  imu.measurement(ImuRoll) = imu.measurement(ImuPitch) =
      imu.measurement(ImuYaw) = 0;
  imu.measurement(ImuVroll) = imu.measurement(ImuVpitch) =
      imu.measurement(ImuVyaw) = 0.0;
  imu.measurement(ImuAx) = imu.measurement(ImuAy) = imu.measurement(ImuAz) =
      0.01;
  imu.covariance.block<3, 3>(ImuRoll, ImuRoll) =
      Eigen::Matrix3d::Identity(3, 3);
  imu.covariance.block<3, 3>(ImuVroll, ImuVroll) =
      Eigen::Matrix3d::Identity(3, 3) * 0.1;
  imu.covariance.block<3, 3>(ImuAx, ImuAx) =
      Eigen::Matrix3d::Identity(3, 3) * 0.05;
  fusion.processMeasurement(imu);

  Measurement vio(VIO_SIZE);
  vio.type = MeasurementTypeVio;
  vio.time = 0.1;  // deltaT = 0.1
  vio.measurement << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;  // x, y, z, vx, vy, vz
  vio.covariance.setZero();
  vio.covariance.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3) * 1e-2;  // position covariance
  vio.covariance.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity(3, 3) * 1e-3;  // velocity covariance
  fusion.processMeasurement(vio);

  // Check that VIO measurement was processed (basic verification)
  EXPECT_TRUE(fusion.isInitialized());
  
  // Verify that the state was updated with VIO data
  Eigen::Matrix<double, STATE_SIZE, 1> state = fusion.getState();
  EXPECT_NEAR(vio.measurement(VioX), state(StateX), 0.1);
  EXPECT_NEAR(vio.measurement(VioY), state(StateY), 0.1);
  EXPECT_NEAR(vio.measurement(VioZ), state(StateZ), 0.1);
  EXPECT_NEAR(vio.measurement(VioVx), state(StateVx), 0.1);
  EXPECT_NEAR(vio.measurement(VioVy), state(StateVy), 0.1);
  EXPECT_NEAR(vio.measurement(VioVz), state(StateVz), 0.1);
}

TEST(FusionTests, vpsMeasurement) {
  Fusion fusion(1e-2, 2, 0);
  auto cov = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity(
                 STATE_SIZE, STATE_SIZE) *
             0.1;
  fusion.setInitialCovariance(cov);
  fusion.reset();

  // initialize fusion class with IMU measurement
  Measurement imu(IMU_SIZE);
  imu.type = MeasurementTypeImu;
  imu.time = 0.0;
  imu.measurement(ImuRoll) = imu.measurement(ImuPitch) =
      imu.measurement(ImuYaw) = 0;
  imu.measurement(ImuVroll) = imu.measurement(ImuVpitch) =
      imu.measurement(ImuVyaw) = 0.0;
  imu.measurement(ImuAx) = imu.measurement(ImuAy) = imu.measurement(ImuAz) =
      0.01;
  imu.covariance.block<3, 3>(ImuRoll, ImuRoll) =
      Eigen::Matrix3d::Identity(3, 3);
  imu.covariance.block<3, 3>(ImuVroll, ImuVroll) =
      Eigen::Matrix3d::Identity(3, 3) * 0.1;
  imu.covariance.block<3, 3>(ImuAx, ImuAx) =
      Eigen::Matrix3d::Identity(3, 3) * 0.05;
  fusion.processMeasurement(imu);

  Measurement vps(VPS_SIZE);
  vps.time = 0.1;  // deltaT = 0.1
  vps.type = MeasurementTypeVps;
  vps.measurement << 5.0, 10.0;  // x, y position
  vps.covariance << 1e-2, 0, 0, 1e-2;
  fusion.processMeasurement(vps);

  // Check that VPS measurement was processed (basic verification)
  EXPECT_TRUE(fusion.isInitialized());
  
  // Verify that the state was updated with VPS data
  Eigen::Matrix<double, STATE_SIZE, 1> state = fusion.getState();
  EXPECT_NEAR(vps.measurement(0), state(StateX), 0.1);
  EXPECT_NEAR(vps.measurement(1), state(StateY), 0.1);
}

TEST(FusionTests, imuMeasurement) {
  Fusion fusion(1e-2, 2, 0);
  auto cov = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity(
                 STATE_SIZE, STATE_SIZE) *
             0.1;
  fusion.setInitialCovariance(cov);
  fusion.reset();

  // initialize fusion class with IMU measurement
  Measurement imu(IMU_SIZE);
  imu.type = MeasurementTypeImu;
  imu.time = 0.0;
  imu.measurement(ImuRoll) = imu.measurement(ImuPitch) =
      imu.measurement(ImuYaw) = 0.5;
  imu.measurement(ImuVroll) = imu.measurement(ImuVpitch) =
      imu.measurement(ImuVyaw) = 0.1;
  imu.measurement(ImuAx) = imu.measurement(ImuAy) = imu.measurement(ImuAz) =
      0.00;
  imu.covariance.block<3, 3>(ImuRoll, ImuRoll) =
      Eigen::Matrix3d::Identity(3, 3) * 1e-2;
  imu.covariance.block<3, 3>(ImuVroll, ImuVroll) =
      Eigen::Matrix3d::Identity(3, 3) * 1e-3;
  imu.covariance.block<3, 3>(ImuAx, ImuAx) =
      Eigen::Matrix3d::Identity(3, 3) * 1e-3;
  fusion.processMeasurement(imu);

  imu.time = 0.1;  // deltaT = 0.1
  imu.type = MeasurementTypeImu;
  imu.measurement << 0.8, 0.8, 0.8, 0.15, 0.15, 0.15, 0.01, 0.01, 0.01;
  fusion.processMeasurement(imu);

  Eigen::Matrix<double, STATE_SIZE, 1> expectedState;
  expectedState << 0, 0, 0, 0.66, 0.65, 0.66, 0, 0, 0, 0.13, 0.13, 0.13, 0.005,
      0.005, 0.005;
  Eigen::Matrix<double, STATE_SIZE, 1> expectedCov;
  expectedCov << 0.15, 0.15, 0.15, 0.04, 0.04, 0.06, 0.125, 0.125, 0.125, 0.01,
      0.01, 0.02, 0.01, 0.01, 0.01;

  for (size_t i = 0; i < STATE_SIZE; i++) {
    EXPECT_NEAR(expectedState(i), fusion.getState()(i), 0.01);
    EXPECT_NEAR(expectedCov(i), fusion.getCovariance().diagonal()(i),
                0.01);
  }
}

// Run all the tests
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}