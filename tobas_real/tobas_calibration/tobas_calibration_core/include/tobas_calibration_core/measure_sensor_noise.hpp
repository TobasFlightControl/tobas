#pragma once

#include <Eigen/Core>
#include <tobas_navio_core/ms5611.hpp>
#include <tobas_navio_core/pwm.hpp>

#include <tobas_navio_ros/common.hpp>

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
  tobas_navio_ros::ImuDevice imu_;
  navio::MS5611 barometer_;
  navio::PWM pwm_;

  void setPeriodOnAllChannels(const double& period);
  void sendDisarm();
  void accelerateMotors();
  void decelerateMotors();
};
}  // namespace tobas_calibration
