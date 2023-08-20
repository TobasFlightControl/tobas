#pragma once

#include <Eigen/Core>
#include <Common/MS5611.h>
#include <Navio2/RCOutput_Navio2.h>

#include "../common.hpp"

namespace tobas_real
{
/* モータを回しながら各センサの白色ノイズの強度を計測する． */
class MeasureSensorNoise
{
  static constexpr uint32_t kDataCount = 1000;
  static constexpr uint32_t kSleepTime = 10000;  // [us]

public:
  explicit MeasureSensorNoise();

  void run();

private:
  ImuDevice imu_;
  MS5611 barometer_;
  RCOutput_Navio2 pwm_;

  void sendDisarm();
};
}  // namespace tobas_real
