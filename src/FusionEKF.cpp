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
FusionEKF::FusionEKF() 
  : is_initialized_(false),
    previous_timestamp_(0),
    noise_ax_(9),
    noise_ay_(9),
    runs_(0)
{
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
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement( SensorPackage & sensorPack ) {
  runs_++;
  /**
   * Initialization
   */
  // The initialize method will call the sensor-specific init method
  // to initialize the state vector. It will then return true, which
  // will cause the ProcessMeasurement method to return immediately
  // on the first run. All subsequent calls to this method
  // will return false, skipping the initialization step and moving on
  // to the processing portion.
  if( initialize(sensorPack) )
    return;

  updateTimeRelatedMatrices( sensorPack.timestamp_ );

  /**
   * Prediction
   */
  // Depending on which descendent class of SensorPackage
  // is passed to this method, the appropriate virtual
  // predict method of that class will be called by the compiler.
  // The Radar and Lidar implementations of SensorPackage both
  // just call the base version of predict (defined in SensorPackage.cpp),
  // but other child classes have the ability to implement their own predict method,
  // which could be useful if data in this phase needs to be linearized.
  sensorPack.predict(motionData_);
  
  /**
   * Update
   */
  // Depending on which descendent class of SensorPackage (Radar or Lidar)
  // is passed to this method, the appropriate virtual
  // updateState method of that class will be called by the compiler.
  sensorPack.updateState(motionData_);

  // print the output
  // cout << "Run: " << runs_ << endl;
  cout << "x_ = " << motionData_.x_ << endl;
  cout << "P_ = " << motionData_.P_ << endl;
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

bool FusionEKF::initialize( SensorPackage & sensorPack ) {
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

    // Depending on which descendent class of SensorPackage
    // is passed to this method, the appropriate virtual
    // initState method of that class will be called by the compiler.
    sensorPack.initState(motionData_);

    previous_timestamp_ = sensorPack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return true;
  }
  else
    return false;
}
