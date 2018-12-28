#ifndef EXTENDEDKALMANFILTER_H_
#define EXTENDEDKALMANFILTER_H_

#include <iostream>

#include "Eigen/Dense"
#include "IKalmanFilter.h"
#include "tools.h"

class ExtendedKalmanFilter 
  : public IKalmanFilter
{
 public:
  /**
   * Constructor
   */
  inline ExtendedKalmanFilter() { }

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  virtual void updateState( MotionData & m );

};

#endif // EXTENDEDKALMANFILTER_H_
