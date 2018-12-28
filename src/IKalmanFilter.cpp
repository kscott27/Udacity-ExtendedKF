#include "IKalmanFilter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

IKalmanFilter::KalmanFilter() {}

IKalmanFilter::~KalmanFilter() {}

void IKalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  // x_ = x_in;
  // P_ = P_in;
  // F_ = F_in;
  // H_ = H_in;
  // R_ = R_in;
  // Q_ = Q_in;
}

void IKalmanFilter::predict( MotionData & m ) {
  /**
   * TODO: predict the state
   */  
  m.x_ = m.F_ * m.x_;
  MatrixXd Ft = m.F_.transpose();
  m.P_ = m.F_ * m.P_ * Ft + m.Q_;
}

