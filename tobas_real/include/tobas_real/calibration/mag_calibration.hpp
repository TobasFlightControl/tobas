#pragma once

#include <Eigen/Core>

#include "../ellipse_transformer.hpp"
#include "../common.hpp"

namespace tobas_real
{
class MagnetometerCalibrator
{
  // Default parameters
  static constexpr char kDefaultMethod[] = "bounding";

  // Constant values
  static constexpr uint32_t kDataCount = 1000;
  static constexpr uint32_t kDirections = 6;
  static constexpr uint32_t kSleepTime = 10000;  // [us]

public:
  explicit MagnetometerCalibrator();

  void run(const std::string& method = kDefaultMethod);

private:
  ImuDevice imu_;

  Eigen::MatrixXd mag_data_;  // 地磁気データのバッファ．メモリ制限回避のため可変サイズで定義．
  EllipseTransformer mag_trans_;

  void getMagData();
  void readMag(uint32_t idx);
};
}  // namespace tobas_real
