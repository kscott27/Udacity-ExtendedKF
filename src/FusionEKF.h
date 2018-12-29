#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "SensorPackage.h"
#include "tools.h"
#include "MotionData.h"

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement( SensorPackage & sensorPack );

  const MotionData & getMotionData() const { return motionData_; }

 private:
  void updateTimeRelatedMatrices( const long long & t );
  bool initialize( SensorPackage & sensorPack );

  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // data object containing all matrices needed for kalman filter
  MotionData motionData_;

  

  // acceleration noise components
  float noise_ax_;
  float noise_ay_;
  uint16_t runs_;
};

#endif // FusionEKF_H_
