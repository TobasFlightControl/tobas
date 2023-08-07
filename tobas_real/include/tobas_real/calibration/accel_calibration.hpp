#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Common/MPU9250.h>

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
  ros::NodeHandle nh_;
  MPU9250 imu_;
  Eigen::Vector3f acc_;

  Eigen::Vector3d readAccel();
};
}  // namespace tobas_real
