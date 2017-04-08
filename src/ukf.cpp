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

    // State vector size
    n_x_ = x_.size();

    // Augmented state vector size
    n_aug_ = n_x_ + 2;

    // Dispersion factor
    lambda_ = 3 - n_aug_;

    // Predicted Sigma Points
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

    // Sigma point weights
    weights_ = VectorXd(2 * n_aug_ + 1);
    InitializeWeights();

    // Predicted radar measurement
    zpred_radar_ = VectorXd(3);

    // Predicted radar measurement covariance
    S_radar_ = MatrixXd(3, 3);

    // Predicted laser measurement
    zpred_laser_ = VectorXd(2);

    // Predicted laser measurement covariance
    S_laser_ = MatrixXd(2, 2);
}

UKF::~UKF() {}

/**
 * Initialize sigma point weights
 */
void UKF::InitializeWeights() {
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < weights_.size(); i++) {
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
    }
}


/**
 * Generate augmented sigma points using current state vector and state covariance matrix.
 */
MatrixXd UKF::GenerateAugmentedSigmaPoints() {
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

    // Generate Augmented Sigma Points
    MatrixXd Xsigma_aug(n_aug_, 2 * n_aug_ + 1);
    Xsigma_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; ++i) {
        Xsigma_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * P_aug_sqrt.col(i);
        Xsigma_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * P_aug_sqrt.col(i);
    }

    return Xsigma_aug;
}

/**
 * Predict Sigma points using state process and noise models.
 *
 */
void UKF::PredictSigmaPoints(const MatrixXd &Xsigma_aug, float dt) {

    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // Extract state from current sigma point.
        double p_x = Xsigma_aug(0, i);
        double p_y = Xsigma_aug(1, i);
        double v = Xsigma_aug(2, i);
        double yaw = Xsigma_aug(3, i);
        double yawd = Xsigma_aug(4, i);
        double nu_a = Xsigma_aug(5, i);
        double nu_yawdd = Xsigma_aug(6, i);

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
        Xsig_pred_(i, 0) = px_p;
        Xsig_pred_(i, 1) = py_p;
        Xsig_pred_(i, 2) = v_p;
        Xsig_pred_(i, 3) = yaw_p;
        Xsig_pred_(i, 4) = yawd_p;
    }
}

/**
 * Predict state using predicted sigma points
 */
void UKF::PredictState() {
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }
}

/**
 * Predict state covariance using sigma points
 */
void UKF::PredictStateCovariance() {
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_.col(i);

        //angle normalization
        while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
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
        UpdateRadar(meas_package);
    } else {
        UpdateLidar(meas_package);
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

    // Generate sigma points using current state vector and state covariance matrix.
    MatrixXd Xsigma_aug = GenerateAugmentedSigmaPoints();

    // Generate predicted sigma points.
    PredictSigmaPoints(Xsigma_aug, delta_t);

    // Predict state vector using predicted sigma points
    PredictState();

    // Predict state covariance using predicted sigma points
    PredictStateCovariance();
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
 * Predict radar measurement and covariance.
 */
void UKF::PredictRadarMeasurement() {

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

    // Compute measurement prediction and measurement covariance prediction
    PredictRadarMeasurement();

    // Update state based on actual radar measurement
}
