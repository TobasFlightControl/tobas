#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Common/MPU9250.h>

namespace tobas_real
{
class MagnetometerCalibrator
{
  static constexpr uint32_t kDataCount = 500;
  static constexpr uint32_t kDirections = 6;
  static constexpr uint32_t kSleepTime = 20000;  // [us]

public:
  explicit MagnetometerCalibrator();

  void run();

private:
  ros::NodeHandle nh_;
  MPU9250 imu_;
  Eigen::Matrix<float, kDataCount * kDirections, 3> mag_;

  void getMagData();
  void readMag(uint32_t idx);

  static bool isValidEllipseCoefs(const Eigen::Matrix<float, 9, 1>& coefs);
};
}  // namespace tobas_real
