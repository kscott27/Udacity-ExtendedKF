#ifndef RADARPACKAGE_H_
#define RADARPACKAGE_H_

#include "SensorPackage.h"
#include "IKalmanFilter.h"

class RadarPackage 
  : public SensorPackage
{
public:
  inline RadarPackage() { }
  virtual inline ~RadarPackage() { }

  virtual void updateState( MotionData & m );
};

#endif // RADARPACKAGE_H_
