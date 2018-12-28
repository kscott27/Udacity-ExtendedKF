#ifndef SENSORPACKAGE_H_
#define SENSORPACKAGE_H_

#include "measurement_package.h"
#include "IKalmanFilter.h"

class SensorPackage 
  : public MeasurementPackage,
    public IKalmanFilter
{
protected:
  inline SensorPackage() { }
  virtual inline ~SensorPackage() { }
};

#endif // SENSORPACKAGE_H_
