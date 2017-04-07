#include <math.h>
#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * Generate augmented sigma points using current state vector and state covariance matrix.
 */
MatrixXd UKF::GenerateAugmentedSigmaPoints() {
    // Size of state vector.
    int n_x = x_.size();
    // Size of augmented state vector.
    int n_aug = n_x + 2;

    // Augmented state vector.
    VectorXd x_aug(n_aug);
    x_aug.setZero();
    x_aug.head(x_.size()) = x_;

    // Augmented state covariance.
    MatrixXd P_aug(n_aug, n_aug);
    p_aug.topLeftCorner(n_x, n_x) = P_;
    p_aug(n_x, n_x) = pow(std_a_, 2);
    p_aug(n_x+1, n_x+1) = pow(std_yawdd_, 2);

    // Generate Augmented Sigma Points
    MatrixXd Xsigma_aug(n_aug, 2*n_aug + 1);
    Xsigma_aug.col(0) = x_aug;
    for (int i=0; i < n_aug; ++i){
        ???
    }
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // Use the first measurement to initialize the filter.
  if (!is_initialized_) {
    // Check the measurement type and initialize appropriatly.
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // With radar can use the measurement to initialize position and velocity, but not pose.
      x_ << meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1)),  // px
              meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(1)),  // py
              meas_package.raw_measurements_(2)  // velocity magnitude (v)
      0,  // yaw angle
              0;  // yaw rate
      // Since the radar tells us nothing about pose, we'll express more uncertainty in that dimension.
      P_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 1, 0, 0,
              0, 0, 0, 10, 0,
              0, 0, 0, 0, 10;

    } else {
      // With laser as the first measurement we can initialize position but not velocity or pose.
      x_ << meas_package.raw_measurements_(0),  // px
              meas_package.raw_measurements_(1),  // py
              0,  // velocity magnitude (v)
              0,  // yaw angle
              0;  // yaw rate
      // Since laser tells us nothing about velocity and pose, we'll express more uncertainty in those dimensions.
      P_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 10, 0, 0,
              0, 0, 0, 10, 0,
              0, 0, 0, 0, 10;
    }
    // Record timestamp of this measurement.
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000;

  Prediction(delta_t);

  /*****************************************************************************
  *  Update
  ****************************************************************************/
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

  } else {

  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
