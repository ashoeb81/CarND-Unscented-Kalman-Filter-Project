#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

#define NUM_RADAR_MEASUREMENTS 3
#define NUM_LIDAR_MEASUREMENTS 2

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    ///* state covariance matrix
    MatrixXd P_;

    ///* sigma points matrix
    MatrixXd Xsigma_;

    ///* augmented sigma points matrix
    MatrixXd Xsigma_aug_;

    ///* time when the state is true, in us
    long long time_us_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    ///* Laser noise covariance
    MatrixXd R_laser_;

    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;

    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_;

    ///* Radar noise covariance
    MatrixXd R_radar_;

    ///* Weights of sigma points
    VectorXd weights_;

    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///* Number of sigma points
    int num_sigma_points_;

    ///* Sigma point spreading parameter
    double lambda_;

    ///* the current NIS for radar
    double NIS_radar_;

    ///* the current NIS for laser
    double NIS_laser_;

    //* Predicted radar measurement
    VectorXd zpred_radar_;

    //* Predicted radar measurement sigma points
    MatrixXd ZSigma_radar_;

    //* Predicted radar measurement covariance
    MatrixXd S_radar_;

    //* Predicted laser measurement
    VectorXd zpred_laser_;

    //* Predicted laser measurement sigma points
    MatrixXd ZSigma_laser_;

    //* Predicted laser measurement covariance
    MatrixXd S_laser_;

    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    /**
     * Initializes weights that will be used to weigh sigma points.
     */
    void InitializeWeights();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);

    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

    /**
     * Generates augmented sigma points using current state vector and
     * state covariance matrix.
     */
    void GenerateSigmaPoints();

    /**
     * Predicts sigma points at time k+1 using state process and noise models
     * @param dt Time elapsed between k and k+1 in seconds.
     */
    void PredictSigmaPoints(float dt);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(MeasurementPackage meas_package);

    /**
     * Predict radar measurement and measurement covariance using predicted
     * sigma points.
    */
    void PredictRadarMeasurement();

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(MeasurementPackage meas_package);

    /**
     * Predict lidar measurement and measurement covariance using predicted
     * sigma points.
     */
    void PredictLidarMeasurement();
};

#endif /* UKF_H */
