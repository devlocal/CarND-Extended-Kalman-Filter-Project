#include "lidar.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace ekf;

Lidar::Lidar(State& state) : state_(state), H_(MatrixXd(2, 4)), R_(MatrixXd(2, 2)), I_(MatrixXd::Identity(4, 4))
{
  // measurement matrix
  H_ <<
    1, 0, 0, 0,
    0, 1, 0, 0;
  Ht_ = H_.transpose();

  // measurement covariance matrix
  R_ <<
    0.0225, 0,
    0, 0.0225;
}

void Lidar::Update(const Eigen::VectorXd &z)
{
  // KF Measurement update step
  VectorXd y = z - H_ * state_.x_;
  MatrixXd S = H_ * state_.P_ * Ht_ + R_;
  MatrixXd K = state_.P_ * Ht_ * S.inverse();
  
  // new state
  state_.x_ = state_.x_ + K * y;
  state_.P_ = (I_ - K * H_) * state_.P_;
}
