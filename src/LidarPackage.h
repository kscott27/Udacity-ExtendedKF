#ifndef LIDARPACKAGE_H_
#define LIDARPACKAGE_H_

#include "SensorPackage.h"

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
