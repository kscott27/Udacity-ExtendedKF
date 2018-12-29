#ifndef RADARPACKAGE_H_
#define RADARPACKAGE_H_

#include "SensorPackage.h"
#include "tools.h"

// A subclass of SensorPackage. This package
// contains raw sensor measurement from a radar,
// as well as a radar-specific implementation of
// the method required to initialize and update
// a Kalman filter, given the MotionData.
class RadarPackage 
  : public SensorPackage
{
public:
  RadarPackage();
  virtual inline ~RadarPackage() { }

  //-- IKalmanFilter pure virtual interface
  virtual void updateState( MotionData & m );
  virtual void initState( MotionData & m );

protected:
  //-- methods
  void angleRangeHandler( float & a );
  void angleJumpHandler( float & a );

  //-- members
  Eigen::MatrixXd Hj_;
  Eigen::MatrixXd R_;
  Tools tools_;
};

#endif // RADARPACKAGE_H_
