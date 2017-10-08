#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "lidar.h"
#include "radar.h"
#include "state.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  const ekf::State& GetState() const { return state_; }

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  ekf::State state_;
  ekf::Radar radar_;
  ekf::Lidar lidar_;

  static const float NOISE_AX;
  static const float NOISE_AY;

  /**
   * Computes initial value of state covariance matrix P
   */
  static Eigen::MatrixXd GetInitialP();

  /**
   * Initializes EKF
   * @param measurement_pack initial measurement
   */
  void Initialize(const MeasurementPackage &measurement_pack);
};

#endif /* FusionEKF_H_ */
