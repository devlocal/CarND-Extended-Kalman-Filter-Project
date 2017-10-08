#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

const float FusionEKF::NOISE_AX = 9;
const float FusionEKF::NOISE_AY = 9;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() : state_(NOISE_AX, NOISE_AY, GetInitialP()), radar_(state_), lidar_(state_) {
  is_initialized_ = false;
  previous_timestamp_ = 0;
}

Eigen::MatrixXd FusionEKF::GetInitialP() {
  MatrixXd P = MatrixXd(4, 4);
  P <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;
  return P;
}

void FusionEKF::Initialize(const MeasurementPackage &measurement_pack) {
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Convert radar measurement from polar to cartesian coordinates
    VectorXd x = ekf::Radar::PolarToCartesian(measurement_pack.raw_measurements_);

    // Initialize state.
    state_.Init(x);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    VectorXd x = VectorXd(4);
    x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    // Initialize state.
    state_.Init(x);
  }
    

  previous_timestamp_ = measurement_pack.timestamp_;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    Initialize(measurement_pack);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  state_.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    radar_.Update(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    lidar_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << state_.x_ << endl;
  cout << "P_ = " << state_.P_ << endl;
}
