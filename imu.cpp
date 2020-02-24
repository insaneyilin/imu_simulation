#include "imu.h"
#include <random>

// euler2Rotation:   body frame to interitail frame
static Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw); double sy = sin(yaw);

    Eigen::Matrix3d RIb;
    RIb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
            -sp,         cp*sr,           cp*cr;
    return RIb;
}

static Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);

    Eigen::Matrix3d R;
    R<<  1,   0,    -sp,
            0,   cr,   sr*cp,
            0,   -sr,  cr*cp;

    return R;
}

namespace imu_simulation {

void IMUSimulation::AddIMUNoise(IMUMotionData *data) {
  Eigen::Vector3d noise_acc;
  GetNormalDistributionNoise(&noise_acc);
  Eigen::Vector3d noise_gyro;
  GetNormalDistributionNoise(&noise_gyro);
  Eigen::Vector3d noise_acc_bias;
  GetNormalDistributionNoise(&noise_acc_bias);
  Eigen::Vector3d noise_gyro_bias;
  GetNormalDistributionNoise(&noise_gyro_bias);

  const double delta_t = 1.0 / imu_params_.frequency;
  const double sqrt_delta_t = std::sqrt(delta_t);

  // Gaussian white noise + Bias noise
  // acc
  Eigen::Matrix3d acc_sqrt_cov = imu_params_.acc_noise_sigma * Eigen::Matrix3d::Identity();
  data->acc += acc_sqrt_cov * noise_acc / sqrt_delta_t + acc_bias_;
  // gyro
  Eigen::Matrix3d gyro_sqrt_cov = imu_params_.gyro_noise_sigma * Eigen::Matrix3d::Identity();
  data->gyro += gyro_sqrt_cov * noise_gyro / sqrt_delta_t + gyro_bias_;

  // update bias
  // acc bias
  acc_bias_ += imu_params_.acc_bias_sigma * sqrt_delta_t * noise_acc_bias;
  data->acc_bias = acc_bias_;
  // gyro bias
  gyro_bias_ += imu_params_.gyro_bias_sigma * sqrt_delta_t * noise_gyro_bias;
  data->gyro_bias = gyro_bias_;
}

void IMUSimulation::GetMotionData(double t, IMUMotionData *data) {
  // see https://github.com/HeYijia/vio_data_simulation/blob/master/src/imu.cpp
  // motion model params
  float ellipse_x = 15;
  float ellipse_y = 20;
  float z = 1;            // z轴做sin运动
  float K1 = 10;          // z轴的正弦频率是x，y的k1倍
  float K = M_PI/ 10;     // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

  // translation
  // twb:  body frame in world frame
  Eigen::Vector3d position( ellipse_x * cos( K * t) + 5, ellipse_y * sin( K * t) + 5,  z * sin( K1 * K * t ) + 5);
  Eigen::Vector3d dp(- K * ellipse_x * sin(K*t),  K * ellipse_y * cos(K*t), z*K1*K * cos(K1 * K * t));              // position导数　in world frame
  double K2 = K*K;
  Eigen::Vector3d ddp( -K2 * ellipse_x * cos(K*t),  -K2 * ellipse_y * sin(K*t), -z*K1*K1*K2 * sin(K1 * K * t));     // position二阶导数

  // Rotation
  double k_roll = 0.1;
  double k_pitch = 0.2;
  Eigen::Vector3d eulerAngles(k_roll * cos(t) , k_pitch * sin(t) , K*t );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
  Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);      // euler angles 的导数

  Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);         // body frame to world frame
  Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

  Eigen::Vector3d gn (0,0,-9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
  Eigen::Vector3d imu_acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

  // fill motion data
  data->timestamp = t;
  data->acc = imu_acc;
  data->gyro = imu_gyro;
  data->velocity = dp;
  data->pose.linear() = Rwb;
  data->pose.translation() = position;
}

void IMUSimulation::GetNormalDistributionNoise(Eigen::Vector3d *noise) const {
  static std::random_device rd;
  static std::default_random_engine generator(rd());
  static std::normal_distribution<double> nd(0.0, 1.0);

  (*noise)[0] = nd(generator);
  (*noise)[1] = nd(generator);
  (*noise)[2] = nd(generator);
}

}  // namespace imu_simulation
