#pragma once

#include <Eigen/Core>
#include <Common/MPU9250.h>
#include <Navio2/LSM9DS1.h>

namespace tobas_real
{
class AccelCalibrator
{
  static constexpr uint32_t kDataCount = 1000;
  static constexpr uint32_t kSleepTime = 10000;  // [us]

public:
  explicit AccelCalibrator();

  void run();

private:
  // MPU9250 imu_;
  LSM9DS1 imu_;

  Eigen::Vector3f acc_;

  Eigen::Vector3f readAccel();
};
}  // namespace tobas_real
