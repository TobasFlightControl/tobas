#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Common/MPU9250.h>

#include "../ellipse_transformer.hpp"

namespace tobas_real
{
class MagnetometerCalibrator
{
  // Default parameters
  const std::string kDefaultMethod = "sphere";

  // Constant values
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
  EllipseTransformer mag_trans_;

  // rosparam
  std::string method_;

  void getRosParams();
  void getMagData();
  void readMag(uint32_t idx);
};
}  // namespace tobas_real
