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
    std_a_ = 3;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 3;

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Laser noise covariance
    R_laser_ = MatrixXd(NUM_LIDAR_MEASUREMENTS, NUM_LIDAR_MEASUREMENTS);
    R_laser_ << pow(std_laspx_, 2), 0,
            0, pow(std_laspy_, 2);

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

    // Radar noise covariance
    R_radar_ = MatrixXd(NUM_RADAR_MEASUREMENTS, NUM_RADAR_MEASUREMENTS);
    R_radar_ << pow(std_radr_, 2), 0, 0,
            0, pow(std_radphi_, 2), 0,
            0, 0, pow(std_radrd_, 2);

    // State vector size
    n_x_ = x_.size();

    // Augmented state vector size
    n_aug_ = n_x_ + 2;

    // Sigma points spreading parameter
    lambda_ = 3 - n_aug_;

    // Number of sigma points
    num_sigma_points_ = 2 * n_aug_ + 1;

    // Sigma points
    Xsigma_ = MatrixXd(n_x_, num_sigma_points_);

    // Augmented sigma points
    Xsigma_aug_ = MatrixXd(n_aug_, num_sigma_points_);

    // Sigma points weights
    weights_ = VectorXd(num_sigma_points_);

    // Predicted radar measurement
    zpred_radar_ = VectorXd(NUM_RADAR_MEASUREMENTS);

    // Predicted radar measurement sigma points
    ZSigma_radar_ = MatrixXd(NUM_RADAR_MEASUREMENTS, num_sigma_points_);

    // Predicted radar measurement covariance
    S_radar_ = MatrixXd(NUM_RADAR_MEASUREMENTS, NUM_RADAR_MEASUREMENTS);

    // Predicted laser measurement
    zpred_laser_ = VectorXd(NUM_LIDAR_MEASUREMENTS);

    // Predicted laser measurement sigma points
    ZSigma_laser_ = MatrixXd(NUM_LIDAR_MEASUREMENTS, num_sigma_points_);

    // Predicted laser measurement covariance
    S_laser_ = MatrixXd(NUM_LIDAR_MEASUREMENTS, NUM_LIDAR_MEASUREMENTS);
}

UKF::~UKF() {}


/**
 * Initialize sigma point weights.
 * @attention Updates the class variable weights_ after invocation.
 */
void UKF::InitializeWeights() {

    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < weights_.size(); i++) {
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
    }
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

    // Use the first measurement to initialize the filter.
    if (!is_initialized_) {

        // Check the measurement type and initialize appropriately.
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

            // With radar can use the measurement to initialize position and velocity, but not pose.
            x_ << meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1)),  // px
                    meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(1)),  // py
                    meas_package.raw_measurements_(2),  // velocity magnitude (v)
                    0,  // yaw angle
                    0;  // yaw rate

            // Since the radar tells us nothing about pose, we'll express more uncertainty in that dimension.
            P_ << 1, 0, 0, 0, 0,
                    0, 1, 0, 0, 0,
                    0, 0, 1, 0, 0,
                    0, 0, 0, 100, 0,
                    0, 0, 0, 0, 100;

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
                    0, 0, 100, 0, 0,
                    0, 0, 0, 100, 0,
                    0, 0, 0, 0, 100;
        }

        // Initialize weights that will be used to weight sigma points
        InitializeWeights();

        // Record timestamp of this measurement.
        time_us_ = meas_package.timestamp_;

        // Mark the filter as being initialized.
        is_initialized_ = true;

        return;
    }

    /*****************************************************************************
    *  Prediction
    ****************************************************************************/

    // Compute time elapsed since last measurement in seconds.
    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000;

    // Update time of last measurement.
    time_us_ = meas_package.timestamp_;

    // Predict sigma points, the state vector, and the state covariance.
    Prediction(delta_t);

    /*****************************************************************************
    *  Update
    ****************************************************************************/

    // Check the type of sensor measurement received and update predicted state
    // appropriately.
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        UpdateRadar(meas_package);
        return;
    }

    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        UpdateLidar(meas_package);
        return;
    }
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

    // Generate sigma points for time k.
    // The generated sigma points will be in the class variable Xsigma_
    GenerateSigmaPoints();

    // Predicted sigma points at time k+1.
    // The predicted sigma points will be in the class variable Xsigma_
    PredictSigmaPoints(delta_t);


    // Use sigma points at time k+1 to compute state vector at k+1.
    x_.fill(0.0);
    for (int i = 0; i < num_sigma_points_; i++) {
        x_ = x_ + weights_(i) * Xsigma_.col(i);
    }


    // Use sigma points at time k+1 to compute state covariance at k+1.
    P_.fill(0.0);
    for (int i = 0; i < num_sigma_points_; i++) {
        VectorXd x_diff = Xsigma_.col(i) - x_;

        // Since x_diff contains the difference of angles, normalize so
        // that the difference is between -pi and pi.
        while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
}

/**
 * Generates augmented sigma points using current state vector and
 * state covariance matrix.
 * @attention Updates the class variable Xsigma_ with the sigma points.
 */
void UKF::GenerateSigmaPoints() {

    // Augmented state vector.
    VectorXd x_aug(n_aug_);
    x_aug.setZero();
    x_aug.head(x_.size()) = x_;

    // Augmented state covariance.
    MatrixXd P_aug(n_aug_, n_aug_);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = pow(std_a_, 2);
    P_aug(n_x_ + 1, n_x_ + 1) = pow(std_yawdd_, 2);

    // Square root of augmented state covariance matrix.
    MatrixXd P_aug_sqrt = P_aug.llt().matrixL();

    // Sigma point calculation
    Xsigma_aug_.col(0) = x_aug;
    for (int i = 0; i < n_aug_; ++i) {
        Xsigma_aug_.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * P_aug_sqrt.col(i);
        Xsigma_aug_.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * P_aug_sqrt.col(i);
    }
}

/**
 * Predicts sigma points at time k+1 using state process and noise models
 * @param dt Time elapsed between k and k+1 in seconds.
 * @attention Updates the class variable Xsigma_ with predicted sigma points.
 */
void UKF::PredictSigmaPoints(float dt) {

    for (int i = 0; i < num_sigma_points_; i++) {

        // Extract state from current sigma point.
        double p_x = Xsigma_aug_(0, i);
        double p_y = Xsigma_aug_(1, i);
        double v = Xsigma_aug_(2, i);
        double yaw = Xsigma_aug_(3, i);
        double yawd = Xsigma_aug_(4, i);

        // Extract noise terms from current sigma point.
        double nu_a = Xsigma_aug_(5, i);
        double nu_yawdd = Xsigma_aug_(6, i);

        // Predict position
        double px_p, py_p;
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v / yawd * (sin(yaw + yawd * dt) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * dt));
        } else {
            px_p = p_x + v * cos(yaw) * dt;
            py_p = p_y + v * sin(yaw) * dt;
        }

        // Predict velocity
        double v_p = v;

        // Predict yaw
        double yaw_p = yaw + yawd * dt;

        // Predict yaw rate
        double yawd_p = yawd;

        // Add noise position
        px_p = px_p + 0.5 * pow(dt, 2) * cos(yaw) * nu_a;
        py_p = py_p + 0.5 * pow(dt, 2) * sin(yaw) * nu_a;

        // Add noise to velocity.
        v_p = v_p + dt * nu_a;

        // Add noise to pose
        yaw_p = yaw_p + 0.5 * pow(dt, 2) * nu_yawdd;
        yawd_p = yawd_p + dt * nu_yawdd;

        // Write predicted sigma point
        Xsigma_(0, i) = px_p;
        Xsigma_(1, i) = py_p;
        Xsigma_(2, i) = v_p;
        Xsigma_(3, i) = yaw_p;
        Xsigma_(4, i) = yawd_p;
    }
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

    // Predict radar measurement using predicted sigma points.
    PredictRadarMeasurement();

    // Update state based on actual radar measurement
    VectorXd x_diff;
    VectorXd z_diff;
    MatrixXd Tc(Xsigma_.rows(), ZSigma_radar_.rows());
    Tc.fill(0.0);
    for (int i = 0; i < Xsigma_.cols(); i++) {

        x_diff = Xsigma_.col(i) - x_;
        while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        z_diff = ZSigma_radar_.col(i) - zpred_radar_;
        while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K;
    MatrixXd K = Tc * S_radar_.inverse();

    // Residual
    z_diff = meas_package.raw_measurements_ - zpred_radar_;

    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S_radar_ * K.transpose();

    // Update radar NIS
    NIS_radar_ = z_diff.transpose() * S_radar_.inverse() * z_diff;

}

/**
 * Predict radar measurement and measurement covariance using predicted
 * sigma points.
 * @attention Modifies the class member Zsigma_radar_
 */
void UKF::PredictRadarMeasurement() {

    // Project each sigma point into the radar measurement space.
    for (int i = 0; i < num_sigma_points_; i++) {
        double px = Xsigma_(0, i);
        double py = Xsigma_(1, i);
        double vx = Xsigma_(2, i) * cos(Xsigma_(3, i));
        double vy = Xsigma_(2, i) * sin(Xsigma_(3, i));
        ZSigma_radar_(0, i) = sqrt(pow(px, 2) + pow(py, 2));
        ZSigma_radar_(1, i) = atan2(py, px);
        ZSigma_radar_(2, i) = (px * vx + py * vy) / ZSigma_radar_(0, i);
    }

    // Compute predicted measurement.
    zpred_radar_.fill(0);
    for (int i = 0; i < ZSigma_radar_.cols(); i++) {
        zpred_radar_ = zpred_radar_ + weights_(i) * ZSigma_radar_.col(i);
    }

    // Compute predicted measurement covariance
    S_radar_.fill(0);
    for (int i = 0; i < ZSigma_radar_.cols(); i++) {
        VectorXd z_diff = ZSigma_radar_.col(i) - zpred_radar_;

        // Since z_diff contains the difference of angles, normalize so
        // that the difference is between -pi and pi.
        while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        S_radar_ = S_radar_ + weights_(i) * z_diff * z_diff.transpose();
    }


    // Add radar sensor noise covariance
    S_radar_ = S_radar_ + R_radar_;
}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

    // Predict laser measurement using predicted sigma points.
    PredictLidarMeasurement();


    // Update state based on actual laser measurement
    VectorXd x_diff;
    VectorXd z_diff;
    MatrixXd Tc(Xsigma_.rows(), ZSigma_laser_.rows());
    Tc.fill(0.0);
    for (int i = 0; i < Xsigma_.cols(); i++) {

        x_diff = Xsigma_.col(i) - x_;
        while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        z_diff = ZSigma_laser_.col(i) - zpred_laser_;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S_laser_.inverse();

    //residual
    z_diff = meas_package.raw_measurements_ - zpred_laser_;

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S_laser_ * K.transpose();

    // Update radar NIS
    NIS_laser_ = z_diff.transpose() * S_laser_.inverse() * z_diff;
}

/*
 * Predict lasesr measurement and covariance
 */
void UKF::PredictLidarMeasurement() {

    // Project each sigma point into the laser measurement space.
    for (int i = 0; i < Xsigma_.cols(); i++) {
        double px = Xsigma_(0, i);
        double py = Xsigma_(1, i);
        ZSigma_laser_(0, i) = px;
        ZSigma_laser_(1, i) = py;
    }

    // Compute predicted measurement.
    zpred_laser_.fill(0);
    for (int i = 0; i < ZSigma_laser_.cols(); i++) {
        zpred_laser_ = zpred_laser_ + weights_(i) * ZSigma_laser_.col(i);
    }

    // Compute predicted measurement covariance
    S_laser_.fill(0);
    VectorXd z_diff;
    for (int i = 0; i < ZSigma_laser_.cols(); i++) {
        z_diff = ZSigma_laser_.col(i) - zpred_laser_;
        S_laser_ = S_laser_ + weights_(i) * z_diff * z_diff.transpose();
    }

    // Add laser sensor noise covariance
    S_laser_ = S_laser_ + R_laser_;
}




