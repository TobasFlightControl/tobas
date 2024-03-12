#pragma once

#include <ros/ros.h>

#include <navio2/adc.hpp>

#include <tobas_calibration_msgs/AdcCalibration.h>

namespace tobas_calibration
{
class AdcCalibrationRos
{
  static constexpr char kServiceName[] = "adc_calibration";

  static constexpr size_t kDataCount = 500;
  static constexpr size_t kSamplingRate = 100;  // [Hz]
  static constexpr double kValidAdcCoefMin = 9.;
  static constexpr double kValidAdcCoefMax = 13.;

  using SrvType = tobas_calibration_msgs::AdcCalibration;

public:
  explicit AdcCalibrationRos(ros::NodeHandle& nh);

private:
  navio::ADC adc_;

  ros::ServiceServer ss_;

  bool executeCb(SrvType::Request& req, SrvType::Response& res);
};
}  // namespace tobas_calibration
