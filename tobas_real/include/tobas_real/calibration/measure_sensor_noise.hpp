#pragma once

#include <Eigen/Core>

#include <Common/MPU9250.h>
#include <Navio2/LSM9DS1.h>
#include <Common/MS5611.h>
#include <Navio2/RCOutput_Navio2.h>

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
  // MPU9250 imu_;
  LSM9DS1 imu_;
  MS5611 barometer_;
  RCOutput_Navio2 pwm_;

  void sendDisarm();
};
}  // namespace tobas_real
