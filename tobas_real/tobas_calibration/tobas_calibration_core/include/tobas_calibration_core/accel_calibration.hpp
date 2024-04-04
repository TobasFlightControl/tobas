#pragma once

#include <Eigen/Core>

#include <tobas_navio_ros/common.hpp>

namespace tobas_calibration
{
class AccelCalibrator
{
  static constexpr size_t kDataCount = 1000;
  static constexpr size_t kSleepTime = 10000;  // [us]

public:
  explicit AccelCalibrator();

  void run();

private:
  tobas_navio_ros::ImuDevice imu_;

  Eigen::Vector3f acc_;

  Eigen::Vector3f readAccel();
};
}  // namespace tobas_calibration
