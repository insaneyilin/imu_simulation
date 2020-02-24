#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace imu_simulation {

struct IMUParams {
  int frequency = 200;
  double start_timestamp = 0.0;  // in seconds
  double duration = 20.0;  // in seconds

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

class IMUSimulation {
 public:
  IMUSimulation() = default;
  ~IMUSimulation() = default;

  void SetIMUParams(const IMUParams &params) {
    imu_params_ = params;
  }

  void SetBiasToZero() {
    acc_bias_ = Eigen::Vector3d::Zero();
    gyro_bias_ = Eigen::Vector3d::Zero();
  }

  void AddIMUNoise(IMUMotionData *data);

  // currently a fixed motion model
  void GetMotionData(double timestamp, IMUMotionData *data);

 private:
  void GetNormalDistributionNoise(Eigen::Vector3d *noise) const;

 private:
  IMUParams imu_params_;

  Eigen::Vector3d acc_bias_;
  Eigen::Vector3d gyro_bias_;
};

}  // namespace imu_simulation
