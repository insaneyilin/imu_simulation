#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <iomanip>

#include <Eigen/Core>
#include "imu.h"

using namespace imu_simulation;

void SavePose(const std::string &filename, const std::vector<IMUMotionData> &data) {
  std::ofstream ofs;
  ofs.setf(std::ios::fixed, std::ios::floatfield);
  ofs.open(filename.c_str());
  for (int i = 0; i < data.size(); ++i) {
    ofs << std::to_string(data[i].pose.translation().x()) << " "
        << std::to_string(data[i].pose.translation().y()) << " "
        << std::to_string(data[i].pose.translation().z()) << std::endl;
  }
}

int main(int argc, char **argv) {
  IMUParams imu_params;
  IMUSimulation imu_sim;
  imu_sim.SetIMUParams(imu_params);

  std::vector<IMUMotionData> imudata;
  std::vector<IMUMotionData> imudata_noise;

  for (float t = imu_params.start_timestamp;
      t < imu_params.start_timestamp + imu_params.duration; t += 1.0 / imu_params.frequency) {
    IMUMotionData data;
    imu_sim.GetMotionData(t, &data);
    imudata.push_back(data);

    IMUMotionData data_noise = data;
    imu_sim.AddIMUNoise(&data_noise);
    imudata_noise.push_back(data_noise);
  }
  SavePose("imu_pose.txt", imudata);
  SavePose("imu_pose_noise.txt", imudata_noise);

  return 0;
}
