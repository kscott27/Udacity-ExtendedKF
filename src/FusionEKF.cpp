#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
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

  motionData_.Rl_ = R_laser_;
  motionData_.Rr_ = R_radar_;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  motionData_.H_ = H_laser_;
  motionData_.Hj_ = Hj_;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  // the initial transition matrix F_
  motionData_.F_ = MatrixXd(4, 4);
  motionData_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  motionData_.P_ = MatrixXd(4,4);
  motionData_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

  noise_ax_ = 9;
  noise_ay_ = 9;
  runs_ = 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement( SensorPackage & sensorPack ) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state motionData_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    motionData_.x_ = VectorXd(4);
    motionData_.x_ << 1, 1, 1, 1;

    sensorPack.initState(motionData_);

    previous_timestamp_ = sensorPack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  updateTimeRelatedMatrices( sensorPack.timestamp_ );
  motionData_.Hj_ = tools.CalculateJacobian(motionData_.x_);

  /**
   * Prediction
   */
  sensorPack.predict(motionData_);
  
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  sensorPack.updateState(motionData_);


  // print the output
  cout << "Run: " << ++runs_ << endl;
  
  cout << "x_ = " << motionData_.x_ << endl;
  // cout << "P_ = " << motionData_.P_ << endl;
}

void FusionEKF::updateTimeRelatedMatrices( const long long & t ) {
  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (t - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = t;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  motionData_.F_(0,2) = dt;
  motionData_.F_(1,3) = dt;

  motionData_.Q_ = MatrixXd(4,4);
  motionData_.Q_ <<  dt_4/4*noise_ax_, 0, dt_3/2*noise_ax_, 0,
         0, dt_4/4*noise_ay_, 0, dt_3/2*noise_ay_,
         dt_3/2*noise_ax_, 0, dt_2*noise_ax_, 0,
         0, dt_3/2*noise_ay_, 0, dt_2*noise_ay_;
}
