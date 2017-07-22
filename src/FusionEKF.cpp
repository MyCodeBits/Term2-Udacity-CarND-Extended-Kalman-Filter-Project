#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    /*
     * Initialize the FusionEKF.
     * Set the process and measurement noises
     */
    // measurement function matrix - laser
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    //state covariance matrix: low values for high certainity
    MatrixXd P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

    // state transition matrix
    MatrixXd F_ = MatrixXd(4, 4);
    F_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;

    // new covariance matrix based on noise vector
    MatrixXd Q_ = MatrixXd(4, 4);
    Q_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          1, 0, 1, 0,
          0, 1, 0, 1;

    VectorXd x_ = VectorXd(4);
    x_ << 1, 1, 1, 1;
    ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
          * Initialize the state ekf_.x_ with the first measurement.
          * Create the covariance matrix.
          * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */
        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        // initialize position and velocity
        float px = 0.0;
        float py = 0.0;
        float vx = 0.0;
        float vy = 0.0;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            // Convert radar from polar to cartesian coordinates and initialize state.
            float rho = measurement_pack.raw_measurements_[0];
            float phi = measurement_pack.raw_measurements_[1];
            float rho_dot = measurement_pack.raw_measurements_[2];

            px = rho * cos(phi);
            py = rho * sin(phi);
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            px = measurement_pack.raw_measurements_[0];
            py = measurement_pack.raw_measurements_[1];
        }

        ekf_.x_ << px, py, vx, vy;
        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
       * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
       * Update the process noise covariance matrix.
       * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */

    // calculating time elapsed
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    // Update the state transition matrix F according to the new elapsed time
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt / 2.0;
    float dt_4 = dt_3 * dt / 2.0;

    //set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4 * noise_ax, 0, dt_3 * noise_ax, 0,
               0, dt_4 * noise_ay, 0, dt_3 * noise_ay,
               dt_3 * noise_ax, 0, dt_2 * noise_ax, 0,
               0, dt_3 * noise_ay, 0, dt_2 * noise_ay;

    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        try {
            ekf_.R_ = R_radar_;
            ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
            ekf_.UpdateEKF(measurement_pack.raw_measurements_);
        } catch (...) {
            return;
        }
    } else {
        // Laser updates
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}