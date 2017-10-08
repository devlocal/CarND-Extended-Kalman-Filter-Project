#ifndef EKF_STATE_H_
#define EKF_STATE_H_

#include "Eigen/Dense"

namespace ekf {

class State {
public:
  // state vector
  Eigen::VectorXd x_;
  
  // state covariance matrix
  Eigen::MatrixXd P_;

public:
  /**
   * Creates Kalman filter state
   * @param noise_ax process noise
   * @param noise_ay process noise
   * @param P Initial state covariance
   */
  State(float noise_ax, float noise_ay, const Eigen::MatrixXd &P);

  /**
   * Initializes Kalman filter state
   * @param x_in Initial state
   */
  void Init(const Eigen::VectorXd &x_in);

  /**
   * Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(float delta_T);

private:
  // process noise
  const float noise_ax_;
  const float noise_ay_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  /**
   * Computes process covariance matrix Q
   * @param delta_T Time between k and k+1 in s
   */
  void ComputeQ(float delta_T);  
};

}

#endif