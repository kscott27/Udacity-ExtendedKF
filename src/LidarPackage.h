#ifndef LIDARPACKAGE_H_
#define LIDARPACKAGE_H_

#include "SensorPackage.h"

// A subclass of SensorPackage. This package
// contains raw sensor measurement from a lidar,
// as well as a lidar-specific implementation of
// the method required to initialize and update
// a Kalman filter, given the MotionData.
class LidarPackage 
  : public SensorPackage
{
public:
  LidarPackage();
  virtual inline ~LidarPackage() { }

  //-- IKalmanFilter pure virtual interface
  virtual void updateState( MotionData & m );
  virtual void initState( MotionData & m );

protected:
  Eigen::MatrixXd R_;
  Eigen::MatrixXd H_;
};

#endif // LIDARPACKAGE_H_
