#ifndef EKF_LIDAR_H_
#define EKF_LIDAR_H_

#include "Eigen/Dense"
#include "state.h"

namespace ekf {

class Lidar {
public:
  Lidar(State& state);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

private:
  State& state_;
  Eigen::MatrixXd I_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd Ht_;
  Eigen::MatrixXd R_;
};

}

#endif