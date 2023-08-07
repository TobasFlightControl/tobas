#pragma once

#include <ros/ros.h>
#include <Navio2/RCInput_Navio2.h>

namespace tobas_real
{
class RCInputCalibrator
{
  static constexpr uint32_t kDataCount = 500;
  static constexpr uint32_t kSleepTime = 10000;  // [us]
  static constexpr double kInfoPeriod = 0.5;     // [s]
  static constexpr double kPeriodMargin = 100;   // [us]

public:
  explicit RCInputCalibrator();

  void run();

private:
  ros::NodeHandle nh_;
  RCInput_Navio2 rcin_;

  double readRCInput(uint32_t channel);
};
}  // namespace tobas_real
