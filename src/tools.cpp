#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  int n_estimations = estimations.size();
  int n_ground_truth = ground_truth.size();

  if (n_estimations == 0) {
    throw std::runtime_error("Cannot compute RMSE for an empty vector.");
  }

  if (n_estimations != n_ground_truth) {
    std::stringstream s;
    s << "Number of estimations ("
      << n_estimations << ") is not equal to the number of ground truth elements ("
      << n_ground_truth << "). Terminating.";
    throw std::runtime_error(s.str());
  }

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  for(int i = 0; i < n_estimations; ++i) {
      VectorXd d = estimations[i] - ground_truth[i];
      d = d.array() * d.array();
        rmse = rmse + d;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3, 4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  if(px + py == 0) {
    throw std::runtime_error("Cannot compute Jacobian matrix when state is at zero point.");    
  }

  float sum_pow2 = px * px + py * py;
  float sqrt_sum_pow2 = sqrt(sum_pow2);
  float pow32 = pow(sum_pow2, 1.5);
  float vxpy = vx * py;
  float vypx = vy * px;
  Hj <<
    px / sqrt_sum_pow2, py / sqrt_sum_pow2, 0, 0,
    -(py / sum_pow2), px / sum_pow2, 0, 0,
    py * (vxpy - vypx) / pow32, px * (vypx - vxpy) / pow32, px / sqrt_sum_pow2, py / sqrt_sum_pow2;

  return Hj;
}
