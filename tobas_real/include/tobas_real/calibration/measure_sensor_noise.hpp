#pragma once

#include <Eigen/Core>
#include <Common/MS5611.h>
#include <Navio2/PWM.h>

#include "../common.hpp"

namespace tobas_real
{
/* モータを回しながら各センサの白色ノイズの強度を計測する． */
class MeasureSensorNoise
{
  static constexpr size_t kDataCount = 1000;
  static constexpr double kMaxThrottle = 0.5;
  static constexpr double kPwmUpDownTime = 5000000;  // [us]
  static constexpr size_t kPwmSleep = 1000;          // [us]

public:
  explicit MeasureSensorNoise();

  void run();

private:
  ImuDevice imu_;
  MS5611 barometer_;
  PWM pwm_;

  void setPeriodOnAllChannels(const double& period);
  void sendDisarm();
  void accelerateMotors();
  void decelerateMotors();
};
}  // namespace tobas_real
