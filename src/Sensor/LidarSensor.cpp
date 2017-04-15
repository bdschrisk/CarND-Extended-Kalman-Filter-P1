#include "LidarSensor.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
* Constructor.
*/
LidarSensor::LidarSensor()
{
	// set measurement matrix
	H_ = MatrixXd(2, 4);
	H_ << 1, 0, 0, 0,
				0, 1, 0, 0;
	// set measurement covariance
	R_ = MatrixXd(2, 2);
	R_ << 0.0225, 0,
				0, 0.0225;
}

/*
* Destructor.
*/
LidarSensor::~LidarSensor() { }

bool LidarSensor::Handles(const MeasurementPackage::SensorType &sensorType)
{
	return (sensorType == MeasurementPackage::SensorType::LASER);
}

void LidarSensor::Update(const VectorXd &state, const MeasurementPackage &measurement)
{
	Vx_ = state[2];
	Vy_ = state[3];
}

/*
* Projects the state into measurement space
* @param state: State to project into measurement space
*/
VectorXd LidarSensor::Project(const VectorXd &state)
{
	VectorXd measurement = VectorXd(2);

	double px = state[0];
	double py = state[1];

	measurement << px, py;

	return measurement;
}

/*
* Dampens the measurement into state space
* @param measurement: Measurement to project into coordinate space
*/
VectorXd LidarSensor::Dampen(const VectorXd &measurement)
{
	VectorXd state = VectorXd(4);
	double px = measurement[0];
	double py = measurement[1];

	state << px, py, Vx_, Vy_;

	return state;
}