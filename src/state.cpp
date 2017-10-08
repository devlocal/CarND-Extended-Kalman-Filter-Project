#include "state.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace ekf;

State::State(float noise_ax, float noise_ay, const Eigen::MatrixXd &P) :
  noise_ax_(noise_ax), noise_ay_(noise_ay), F_(MatrixXd(4, 4)), Q_(MatrixXd(4, 4))
{
  // state covariance matrix
  P_ = P;

  // state transition matrix
  F_ <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;

  // process covariance matrix
  Q_ <<
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;
}

void State::Init(const Eigen::VectorXd &x_in) {
  x_ = x_in;
}

void State::Predict(float delta_T) {
  // Update delta_t in state transition matrix
  F_(0, 2) = delta_T;
  F_(1, 3) = delta_T;

  ComputeQ(delta_T);
  
  // Compute predicted state
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void State::ComputeQ(float delta_T) {
  float dt4 = pow(delta_T, 4) / 4;
  float dt3 = pow(delta_T, 3) / 2;
  float dt2 = pow(delta_T, 2);

  float dt3_noise_ax = dt3 * noise_ax_;
  float dt3_noise_ay = dt3 * noise_ay_;

  Q_(0, 0) = dt4 * noise_ax_;
  Q_(0, 2) = dt3_noise_ax;
  Q_(1, 1) = dt4 * noise_ay_;
  Q_(1, 3) = dt3_noise_ay;
  Q_(2, 0) = dt3_noise_ax;
  Q_(2, 2) = dt2 * noise_ax_;
  Q_(3, 1) = dt3_noise_ay;
  Q_(3, 3) = dt2 * noise_ay_;
}
