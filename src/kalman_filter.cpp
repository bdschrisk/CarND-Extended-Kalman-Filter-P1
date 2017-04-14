#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter(const int state_dim, const float p_val)
{ 
	// initialise Kalman Filter
	x_ = VectorXd::Ones(state_dim);

	F_ = MatrixXd::Identity(state_dim, state_dim);
	F_.topRightCorner(state_dim / 2, state_dim / 2).setIdentity();
	Q_ = MatrixXd(state_dim, state_dim);

	P_ = MatrixXd::Identity(state_dim, state_dim);
	// set initial uncertainty
	P_.bottomRightCorner(state_dim / 2, state_dim / 2) = MatrixXd::Identity(state_dim / 2, state_dim / 2) * p_val;
}

KalmanFilter::~KalmanFilter() { }

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
}

void KalmanFilter::SetDelta(double delta, double noise_ax, double noise_ay)
{
	// update delta
	F_(0, 2) = delta;
	F_(1, 3) = delta;

	// precompute G params
	double dt_sq = (delta * delta) / 2.0;
	MatrixXd G = MatrixXd(4, 2);
	G <<	dt_sq, 0,
				0, dt_sq,
				delta, 0,
				0, delta;

	// Initialise noise process matrix
	MatrixXd Qv = MatrixXd(2, 2);
	Qv << noise_ax, 0,
				0, noise_ay;
	// increase noise corruption w.r.t time delta
	Q_ = (G * Qv) * G.transpose();
}

void KalmanFilter::Predict(const VectorXd Fx, const MatrixXd Fj) {
	// predict new state and uncertainty
	x_ = Fx;
	MatrixXd F_t  = Fj.transpose();
	P_ = Fj * P_ * F_t + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &H, const MatrixXd &R) {
	// measurement error
	VectorXd y = z - z_pred;
	
	// update step
	MatrixXd Ht = H.transpose();
	MatrixXd PHt = P_ * Ht;
	MatrixXd S = H * PHt + R;
	MatrixXd K = PHt * S.inverse();
	
	// projected update
	x_ = x_ + (K * y);

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H) * P_;
}

