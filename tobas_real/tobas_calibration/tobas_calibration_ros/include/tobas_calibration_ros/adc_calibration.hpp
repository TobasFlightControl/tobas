#pragma once

#include <tobas_algorithm/kahan.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_hal_msgs/Adc.h>
#include <tobas_calibration_msgs/AdcCalibration.h>

namespace tobas_calibration
{
class AdcCalibrationRos : public tobas::BaseNode
{
  static constexpr char kServiceName[] = "adc_calibration";

  static constexpr size_t kDataCount = 100;
  static constexpr double kTimeout = 5.;  // [s]

  using super = tobas::BaseNode;
  using SrvType = tobas_calibration_msgs::AdcCalibration;

public:
  explicit AdcCalibrationRos(
    ros::NodeHandle& nh,
    ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  size_t cnt_;
  algo::Kahan<double> voltage_sum_;

  ptree::PropertyClient property_client_;

  ros::Subscriber adc_sub_;
  ros::ServiceServer ss_;

  void adcCb(const tobas_hal_msgs::AdcConstPtr& adc);
  bool executeCb(SrvType::Request& req, SrvType::Response& res);
};
}  // namespace tobas_calibration
