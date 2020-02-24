#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace imu_simulation {

struct IMUParams {
  int frequency = 200;
  double start_timestamp = 0.0;  // in seconds
  double duration = 10.0;  // in seconds

  double acc_noise_sigma = 0.02;  //　m/(s^2) * 1/sqrt(hz)
  double gyro_noise_sigma = 0.02;  // rad/s * 1/sqrt(hz)

  double acc_bias_sigma = 1.0e-4;
  double gyro_bias_sigma = 1.0e-5;
};

struct IMUMotionData {
  double timestamp = 0.0;

  Eigen::Affine3d pose;  // sensor to world pose

  Eigen::Vector3d acc;
  Eigen::Vector3d gyro;

  Eigen::Vector3d acc_bias;
  Eigen::Vector3d gyro_bias;

  Eigen::Vector3d velocity;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



}  // namespace imu_simulation
