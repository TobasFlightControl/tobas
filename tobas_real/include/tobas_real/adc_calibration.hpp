#pragma once

#include <ros/ros.h>
#include <Navio2/ADC_Navio2.h>

namespace tobas_real
{
class AdcCalibrator
{
  static constexpr uint32_t kDataCount = 500;
  static constexpr double kSleepTime = 1e-2;  // [s]
  static constexpr double kValidAdcCoefMin = 9.;
  static constexpr double kValidAdcCoefMax = 13.;

public:
  explicit AdcCalibrator();

  void run();

private:
  ros::NodeHandle nh_;
  ADC_Navio2 adc_;
};
}  // namespace tobas_real
