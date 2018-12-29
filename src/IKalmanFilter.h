#ifndef IKALMANFILTER_H_
#define IKALMANFILTER_H_

#include <iostream>

#include "Eigen/Dense"
#include "MotionData.h"

// This is a pure virtual class. It implements a predict method
// which can be overridden by child classes. It also has a pure virtual
// update and init method, which must be overridden to be instantiable.
// SensorPackage inherits from this class, yet saves the implementation
// of these methods for specific sensors, which is why SensorPackage is
// a pure virtual interface.
class IKalmanFilter {
 public:
  /**
   * Constructor
   */
  IKalmanFilter();

  /**
   * Destructor
   */
  virtual ~IKalmanFilter();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  virtual void predict( MotionData & m );

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  virtual void updateState( MotionData & m ) = 0;
  virtual void initState( MotionData & m ) = 0;

};

#endif // IKALMANFILTER_H_
