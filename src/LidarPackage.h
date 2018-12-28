#ifndef LIDARPACKAGE_H_
#define LIDARPACKAGE_H_

#include "SensorPackage.h"
#include "kalman_filter.h"

class LidarPackage 
  : public SensorPackage
{
public:
  inline LidarPackage() { }
  virtual inline ~LidarPackage() { }

  virtual void updateState( MotionData & m );
};

#endif // LIDARPACKAGE_H_
