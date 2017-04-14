#include "RadarSensor.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
* Constructor.
*/
RadarSensor::RadarSensor()
{
	H_ = MatrixXd(3, 4);

	R_ = MatrixXd(3, 3);
	R_ << 0.09, 0, 0,
				0, 0.0009, 0,
				0, 0, 0.09;
}

/*
* Destructor.
*/
RadarSensor::~RadarSensor() { }

bool RadarSensor::Handles(const MeasurementPackage::SensorType &sensorType)
{
	return (sensorType == MeasurementPackage::SensorType::RADAR);
}

void RadarSensor::Update(const VectorXd &state, const MeasurementPackage &measurement)
{
	double px = state[0];
	double py = state[1];
	double vx = state[2];
	double vy = state[3];

	// compute first order derivs
	double px_2 = pow(px, 2);
	double py_2 = pow(py, 2);
	double pxpy_2 = (px_2 + py_2);
	double pxpy_3 = pow(pxpy_2, 3 / 2);
	double pxpy_sqrt = sqrt(pxpy_2);

	//check division by zero
	if (pxpy_2 == 0 || pxpy_3 == 0)
		return;

	// compute derivatives
	double dx_px = (px / pxpy_sqrt);
	double dx_py = (py / pxpy_sqrt);
	double dx_phi_px = -(py / pxpy_2);
	double dx_phi_py = (px / pxpy_2);
	double dx_rp_px = ((py * (vx*py - vy*px)) / pxpy_3);
	double dx_rp_dy = ((px * (vy*px - vx*py)) / pxpy_3);

	H_ << dx_px, dx_py, 0, 0,
				dx_phi_px, dx_phi_py, 0, 0,
				dx_rp_px, dx_rp_dy, dx_px, dx_py;
}

/*
* Projects the state into measurement space
* @param state: State to project into measurement space
*/
VectorXd RadarSensor::Project(const VectorXd &state)
{
	VectorXd measurement = VectorXd(3);

	double px = state[0];
	double py = state[1];
	double vx = state[2];
	double vy = state[3];

	// project into polar coordinates
	double rho = sqrt((px*px) + (py*py));
	double phi = atan2(py, px);
	double rho_dot = ((px*vx) + (py*vy)) / rho;

	measurement << rho, phi, rho_dot;

	return measurement;
}

/*
* Dampens the measurement into state space
* @param measurement: Measurement to project into coordinate space
*/
VectorXd RadarSensor::Dampen(const VectorXd &measurement)
{
	VectorXd state = VectorXd(4);

	double rho = measurement[0];
	double phi = measurement[1];
	double rho_dot = measurement[2];

	// project into cartesian coordinates
	double px = rho * cos(phi);
	double py = rho * sin(phi);
	double vx = rho_dot * cos(phi);
	double vy = rho_dot * sin(phi);

	state << px, py, vx, vy;

	return state;
}
