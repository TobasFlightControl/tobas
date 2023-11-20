#pragma once

#include <Eigen/Core>

#include "../common.hpp"

namespace tobas_real
{
class AccelCalibrator
{
  static constexpr size_t kDataCount = 1000;
  static constexpr size_t kSleepTime = 10000;  // [us]

public:
  explicit AccelCalibrator();

  void run();

private:
  ImuDevice imu_;

  Eigen::Vector3f acc_;

  Eigen::Vector3f readAccel();
};
}  // namespace tobas_real
