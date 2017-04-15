#include "FusionEKF.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  noise_ax = 9.0F;
  noise_ay = 9.0F;

	time_delta = 0.001F;

	// Initialize Sensors //
	
	sensors_.push_back(&radar);
	sensors_.push_back(&lidar);

	// Initialise Kalman Filter
	ekf_ = KalmanFilter(4, 1000.0F);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

	// SENSOR
	Sensor *sensor;

	size_t N = sensors_.size();
	for (size_t i = 0; i < N; i++) {
		if (sensors_[i]->Handles(measurement_pack.sensor_type_)) {
			sensor = sensors_[i];
			break;
		}
	}

	// check valid sensor
	if (sensor == NULL) {
		cout << "ProcessMeasurement() - Error: Unknown sensor type '" << measurement_pack.sensor_type_ << "'";
		return;
	}

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;

		// update sensor with initial measurement
		sensor->Update(ekf_.x_, measurement_pack);
		ekf_.x_ = sensor->Dampen(measurement_pack.raw_measurements_);
		// store previous timestamp
		previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  // compute the time elapsed between the current and previous measurements
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	// in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  // update time delta in KF
  ekf_.SetDelta(dt, noise_ax, noise_ay);

	// update sensor with new measurements
	sensor->Update(ekf_.x_, measurement_pack);

	/*****************************************************************************
	*  EKF Predict and Update
	****************************************************************************/

	// check for multiple simultaneous measurements
	if (dt > time_delta)
	{
		// Predicts state P
		ekf_.Predict();
	}

	// check numerical stability
	if ((ekf_.x_[0] > FLT_EPSILON || ekf_.x_[0] < -FLT_EPSILON)
		&& (ekf_.x_[1] > FLT_EPSILON || ekf_.x_[1] < -FLT_EPSILON)) {
		// measurement
		VectorXd z = measurement_pack.raw_measurements_;
		// Updates new state with correct error measures
		VectorXd z_pred = sensor->Project(ekf_.x_);
		ekf_.Update(z, z_pred, sensor->H_, sensor->R_);
	}

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
