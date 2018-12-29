#ifndef SENSORPACKAGE_H_
#define SENSORPACKAGE_H_

#include "measurement_package.h"
#include "IKalmanFilter.h"

// This is an interface class, which any type of sensor
// can inherit from. The inheriting class must implement
// its own specific update and init methods in order to
// be instantiable. The point of this abstract interface class
// is to allow for run-time binding of any type of SensorPackage
// (radar or lidar in this case) when passed to FusionEKF::ProcessMeasurement.
class SensorPackage 
  : public MeasurementPackage,
    public IKalmanFilter
{
protected:
  inline SensorPackage() { }
  virtual inline ~SensorPackage() { }
};

#endif // SENSORPACKAGE_H_
