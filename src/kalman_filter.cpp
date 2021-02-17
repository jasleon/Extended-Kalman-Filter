#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // Update state and covariance
  x_ = x_ + K * y;
  size_t x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  // Recover state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  // Map from cartesian to polar coordinates
  double rho = hypot(px, py);
  double phi = atan2(px, py);
  double rho_dot = (px * vx + py * vy) / std::max(rho, 0.0001);

  // Predicted state in polar coordinates
  VectorXd z_check(3);
  z_check << rho, phi, rho_dot;

  // Calculate the measurement residual
  VectorXd y = z - z_check;

  // Normalize angle
  while(y(1) >  M_PI) y(1) -= 2 * M_PI;
  while(y(1) < -M_PI) y(1) += 2 * M_PI;

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // Update state and covariance
  x_ = x_ + K * y;
  size_t x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
