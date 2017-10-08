#ifndef EKF_RADAR_H_
#define EKF_RADAR_H_

#include "Eigen/Dense"
#include "state.h"

namespace ekf {

class Radar {
public:
  Radar(State& state);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Converts radar measurement from polar to cartesian coordinates.
   * @param x Radar measurement
   */
  static Eigen::VectorXd PolarToCartesian(const Eigen::VectorXd &x);

private:
  State& state_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd I_;
  const double pi_;
  
  /**
   * State transition function
   * @param x State vector
   */
  Eigen::VectorXd h(const Eigen::VectorXd &x);
};

}

#endif