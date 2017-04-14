#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  /**
   * Constructor
   */
	KalmanFilter(const int state_dim = 4, const float p_val = 1000.0);

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in, Eigen::MatrixXd &Q_in);

  /*
  * Sets the time delta of the transition state function
  * @param delta: Time delta expressed in seconds
  * @param noise_ax: X dimensional noise
  * @param noise_ay: Y dimensional noise
  */
  void SetDelta(double delta, double noise_ax, double noise_ay);

  /**
  * Predicts the state and the state covariance
  * @param Fx: State prime
  * @param Fj: State transition matrix
  */
  void Predict(const Eigen::VectorXd Fx, const Eigen::MatrixXd Fj);

  /**
  * Updates the state by using Kalman Filter equations
  * @param z: The measurement at k+1
  * @param z_pred: State transition
  * @param H: Measurement space error matrix
	* @param R: Measurement noise corruption
  */
  void Update(const Eigen::VectorXd &z, const Eigen::VectorXd &z_pred, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);

};

#endif /* KALMAN_FILTER_H_ */
