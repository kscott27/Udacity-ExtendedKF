#ifndef RADARPACKAGE_H_
#define RADARPACKAGE_H_

#include "SensorPackage.h"

class RadarPackage 
  : public SensorPackage
{
public:
  inline RadarPackage() { }
  virtual inline ~RadarPackage() { }

  //-- IKalmanFilter pure virtual interface
  virtual void updateState( MotionData & m );
  virtual void initState( MotionData & m );

protected:
  void angleRangeHandler( float & a );
  void angleJumpHandler( float & a );
};

#endif // RADARPACKAGE_H_
