#include <cmath>
#include "radar.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace ekf;

Radar::Radar(State& state) : pi_(4 * atan(1)), state_(state), R_(MatrixXd(3, 3)), I_(MatrixXd::Identity(4, 4)) {
  // measurement covariance matrix
  R_ <<
    0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
}

Eigen::VectorXd Radar::PolarToCartesian(const Eigen::VectorXd &x) {
  VectorXd out = VectorXd(4);

  float range = x[0];
  float bearing = x[1];
  float px = range * cos(bearing);
  float py = range * sin(bearing);

  out << px, py, 0, 0;

  return out;
}

Eigen::VectorXd Radar::h(const Eigen::VectorXd& x) {
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);

  VectorXd res = VectorXd(3);
  
  float px2_py2 = sqrt(px * px + py * py);

  res <<
    px2_py2,
    px == 0 ? 0 : atan2(py, px),
    px + py == 0 ? 0 : (px * vx + py * vy) / px2_py2;
  return res;
}

void Radar::Update(const Eigen::VectorXd &z) {
  // If state is at zero, Jacobian matrix cannot be computed
  // Take the measurement as is and return.
  if (state_.x_[0] + state_.x_[1] == 0) {
    state_.x_ = PolarToCartesian(z);
    return;
  }

  MatrixXd Hj_ = Tools::CalculateJacobian(state_.x_);
  
  // KF Measurement update step
  VectorXd y = z - h(state_.x_);

  // Normalize angle
  while (y(1) > pi_) {
    y(1) = y(1) - 2 * pi_;
  }

  while (y(1) < -pi_) {
    y(1) = y(1) + 2 * pi_;
  }

  MatrixXd S = Hj_ * state_.P_ * Hj_.transpose() + R_;
  MatrixXd K = state_.P_ * Hj_.transpose() * S.inverse();
  
  // new state
  state_.x_ = state_.x_ + K * y;
  state_.P_ = (I_ - K * Hj_) * state_.P_;
}
