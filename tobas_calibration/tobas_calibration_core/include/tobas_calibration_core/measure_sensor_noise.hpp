#pragma once

#include <Eigen/Core>
#include <navio2/MS5611.h>
#include <navio2/PWM.h>

#include <tobas_real/common.hpp>

namespace tobas_calibration
{
/* モータを回しながら各センサの白色ノイズの強度を計測する． */
class MeasureSensorNoise
{
  static constexpr size_t kDataCount = 1000;
  static constexpr double kMaxThrottle = 0.5;
  static constexpr double kPwmUpDownTime = 5.;  // [s]
  static constexpr size_t kPwmSleep = 1000;     // [us]

public:
  explicit MeasureSensorNoise();

  void run();

private:
  tobas_real::ImuDevice imu_;
  navio::MS5611 barometer_;
  navio::PWM pwm_;

  void setPeriodOnAllChannels(const double& period);
  void sendDisarm();
  void accelerateMotors();
  void decelerateMotors();
};
}  // namespace tobas_calibration
