#pragma once

#include <ros/ros.h>

#include <tobas_std_tools/rate.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_navio_core/adc.hpp>
#include <tobas_calibration_msgs/AdcCalibration.h>

namespace tobas_calibration
{
class AdcCalibrationRos : public tobas::BaseNode
{
  static constexpr char kServiceName[] = "adc_calibration";

  static constexpr size_t kDataCount = 500;
  static constexpr size_t kSamplingRate = 100;  // [Hz]
  static constexpr double kValidAdcCoefMin = 9.;
  static constexpr double kValidAdcCoefMax = 13.;

  using super = tobas::BaseNode;
  using SrvType = tobas_calibration_msgs::AdcCalibration;

public:
  explicit AdcCalibrationRos(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  navio::ADC adc_;
  ptree::PropertyClient property_client_;
  tobas_std::Rate rate_;

  ros::ServiceServer ss_;

  bool executeCb(SrvType::Request& req, SrvType::Response& res);
};
}  // namespace tobas_calibration
