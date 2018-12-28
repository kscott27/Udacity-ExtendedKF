#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <iostream>

#include "IKalmanFilter.h"

class KalmanFilter 
  : public IKalmanFilter
{
 public:

  inline KalmanFilter() { }

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  virtual void updateState( MotionData & m );

};

#endif // KALMAN_FILTER_H_
