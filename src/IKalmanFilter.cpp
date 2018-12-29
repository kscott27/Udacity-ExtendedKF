#include "IKalmanFilter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

IKalmanFilter::IKalmanFilter() {}

IKalmanFilter::~IKalmanFilter() {}

void IKalmanFilter::predict( MotionData & m ) {
  /**
   * TODO: predict the state
   */  
  m.x_ = m.F_ * m.x_;
  MatrixXd Ft = m.F_.transpose();
  m.P_ = m.F_ * m.P_ * Ft + m.Q_;
}


