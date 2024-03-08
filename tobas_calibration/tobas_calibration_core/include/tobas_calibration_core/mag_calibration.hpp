#pragma once

#include <Eigen/Core>

#include <tobas_real/ellipse_transformer.hpp>
#include <tobas_real/common.hpp>

namespace tobas_calibration
{
class MagnetometerCalibrator
{
  // Default parameters
  static constexpr char kDefaultMethod[] = "bounding";

  // Constant values
  static constexpr size_t kDataCount = 1000;
  static constexpr size_t kDirections = 6;
  static constexpr size_t kSleepTime = 10000;  // [us]

public:
  explicit MagnetometerCalibrator();

  void run(const std::string& method = kDefaultMethod);

private:
  tobas_real::ImuDevice imu_;

  Eigen::MatrixXd mag_data_;  // 地磁気データのバッファ．メモリ制限回避のため可変サイズで定義．
  tobas_real::EllipseTransformer mag_trans_;

  void getMagData();
  void readMag(const size_t& idx);
};
}  // namespace tobas_calibration
