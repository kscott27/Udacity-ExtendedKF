#ifndef LIDARPACKAGE_H_
#define LIDARPACKAGE_H_

#include "SensorPackage.h"
#include "kalman_filter.h"

class LidarPackage 
  : public SensorPackage,
    public KalmanFilter
{
public:
  inline LidarPackage() { }
  virtual inline ~LidarPackage() { }
};

#endif // LIDARPACKAGE_H_
