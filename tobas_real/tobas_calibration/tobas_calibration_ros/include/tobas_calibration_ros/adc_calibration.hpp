#pragma once

#include <tobas_algorithm/kahan.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_hal_msgs/msg/adc.hpp>
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
  explicit AdcCalibrationRos(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  size_t cnt_;
  algo::Kahan<double> voltage_sum_;

  ptree::PropertyClient::SharedPtr property_client_;
  ServicePtr<> ss_;

  void adcCb(const tobas_hal_msgs::msg::Adc::ConstSharedPtr& adc);
  bool executeCb(SrvType::Request& req, SrvType::Response& res);
};
}  // namespace tobas_calibration
