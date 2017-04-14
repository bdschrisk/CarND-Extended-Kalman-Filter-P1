#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cfloat>

#include "kalman_filter.h"
#include "tools.h"
#include "Sensor/Sensor.h"
#include "Sensor/LidarSensor.h"
#include "Sensor/RadarSensor.h"

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
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  //acceleration noise components
  float noise_ax;
  float noise_ay;

	float time_delta;

  // tool object used to compute Jacobian and RMSE
  Tools tools;

	// Sensors array
	RadarSensor radar;
	LidarSensor lidar;

	std::vector<Sensor*> sensors_;
};

#endif /* FusionEKF_H_ */
