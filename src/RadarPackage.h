#ifndef RADARPACKAGE_H_
#define RADARPACKAGE_H_

#include "SensorPackage.h"
#include "tools.h"

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
