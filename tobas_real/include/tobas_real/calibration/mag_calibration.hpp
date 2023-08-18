#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Common/MPU9250.h>
#include <Navio2/LSM9DS1.h>

#include "../ellipse_transformer.hpp"

namespace tobas_real
{
class MagnetometerCalibrator
{
  // Default parameters
  const std::string kDefaultMethod = "bounding";

  // Constant values
  static constexpr uint32_t kDataCount = 1000;
  static constexpr uint32_t kDirections = 6;
  static constexpr uint32_t kSleepTime = 10000;  // [us]

public:
  explicit MagnetometerCalibrator();

  void run();

private:
  ros::NodeHandle nh_;

  // MPU9250 imu_;
  LSM9DS1 imu_;

  Eigen::MatrixXd mag_data_;  // 地磁気データのバッファ．メモリ制限回避のため可変サイズで定義．
  EllipseTransformer mag_trans_;

  // rosparam
  std::string method_;

  void getRosParams();
  void getMagData();
  void readMag(uint32_t idx);
};
}  // namespace tobas_real
