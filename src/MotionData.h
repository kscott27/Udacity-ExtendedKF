#ifndef MOTIONDATA_H_
#define MOTIONDATA_H_

#include "Eigen/Dense"

class MotionData {
 public:
  /**
   * Constructor
   */
  inline MotionData() { }

  /**
   * Destructor
   */
  virtual inline ~MotionData() { }

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  long long previousTimestamp_;
};

#endif // MOTIONDATA_H_
