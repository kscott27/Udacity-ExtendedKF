#include "LidarPackage.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void LidarPackage::updateState( MotionData & m ) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = m.H_ * m.x_;
  VectorXd y = rawMeasurements_ - z_pred;
  MatrixXd Ht = m.H_.transpose();
  MatrixXd S = m.H_ * m.P_ * Ht + m.Rl_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = m.P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  m.x_ = m.x_ + (K * y);
  long x_size = m.x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  m.P_ = (I - K * m.H_) * m.P_;
}
