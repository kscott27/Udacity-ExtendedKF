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
  float phiMeasured = rawMeasurements_(1);
  angleRangeHandler( phiMeasured );
  rawMeasurements_(1) = phiMeasured;
  VectorXd y = rawMeasurements_ - z_pred;
  float phiError = y(1);
  angleJumpHandler( phiError );
  y(1) = phiError;
  MatrixXd Hjt = m.Hj_.transpose();
  MatrixXd S = m.Hj_ * m.P_ * Hjt + m.Rr_;
  MatrixXd Si = S.inverse();
  MatrixXd PHjt = m.P_ * Hjt;
  MatrixXd K = PHjt * Si;

  //new estimate
  m.x_ = m.x_ + (K * y);
  long x_size = m.x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  m.P_ = (I - K * m.Hj_) * m.P_;
}

void RadarPackage::initState( MotionData & m ) {
  float rho = rawMeasurements_[0];
  float phi = rawMeasurements_[1];
  float rho_dot = rawMeasurements_[2];
  float px = rho * cos(phi);
  float py = rho * sin(phi);
  float vx = rho_dot * cos(phi);
  float vy = rho_dot * sin(phi);
  m.x_ << px, py, vx, vy;
}

void RadarPackage::angleRangeHandler( float & a ) {
  while( a > 3.14 )
    a = a - 6.28;
  while( a < -3.14 )
    a = a + 6.28;
}

void RadarPackage::angleJumpHandler( float & a ) {
  if( a > 3.14 )
    a = a - 6.28;
  else if( a < -3.14 )
    a = a + 6.28;
}
