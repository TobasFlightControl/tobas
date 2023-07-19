#pragma once

#include <ros/ros.h>
#include <Navio2/ADC_Navio2.h>

namespace tobas_real
{
class AdcCalibrator
{
  static constexpr uint32_t kDataCount = 500;
  static constexpr uint32_t kSleepTime = 10000;  // [us]
  static constexpr double kInfoPeriod = 0.5;     // [s]
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
