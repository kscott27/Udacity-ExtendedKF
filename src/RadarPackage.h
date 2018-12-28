#ifndef RADARPACKAGE_H_
#define RADARPACKAGE_H_

#include "SensorPackage.h"
#include "ExtendedKalmanFilter.h"

class RadarPackage 
  : public SensorPackage,
    public ExtendedKalmanFilter
{
public:
  inline RadarPackage() { }
  virtual inline ~RadarPackage() { }
};

#endif // RADARPACKAGE_H_
