#include "RadarPackage.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

void RadarPackage::updateState( MotionData & m ) {
  float px = m.x_(0);
  float py = m.x_(1);
  float vx = m.x_(2);
  float vy = m.x_(3);
  float rho = sqrt( px*px + py*py );
  float phi = atan2( py, px );
  float rho_dot = ( px*vx + py*vy ) / rho;
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = rawMeasurements_ - z_pred;
  float phi_error = y(1);
  if( phi_error > 6.28 )
    phi_error = phi_error - 6.28;
  else if( phi_error < -6.28 )
    phi_error = phi_error + 6.28;
  y(1) = phi_error;
  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Hjt + Rr_;
  MatrixXd Si = S.inverse();
  MatrixXd PHjt = P_ * Hjt;
  MatrixXd K = PHjt * Si;

  //new estimate
  m.x_ = m.x_ + (K * y);
  long x_size = m.x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  m.P_ = (I - K * m.Hj_) * m.P_;
}
