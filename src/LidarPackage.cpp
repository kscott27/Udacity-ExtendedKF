#include "LidarPackage.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

LidarPackage::LidarPackage() {
  // initializing matrices
  R_ = MatrixXd(2, 2);
  
  H_ = MatrixXd(2, 4);
  

  //measurement covariance matrix - laser
  R_ << 0.0225, 0,
        0, 0.0225;

  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;
}

void LidarPackage::updateState( MotionData & m ) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * m.x_;
  VectorXd y = rawMeasurements_ - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * m.P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = m.P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  m.x_ = m.x_ + (K * y);
  long x_size = m.x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  m.P_ = (I - K * H_) * m.P_;
}

// Initializes the state vector. This method is called
// by FusionEKF::ProcessMeasurement if the state vector
// has not yet been initialized
void LidarPackage::initState( MotionData & m ) {
  m.x_ << rawMeasurements_[0],
          rawMeasurements_[1],
          0,
          0;
}
