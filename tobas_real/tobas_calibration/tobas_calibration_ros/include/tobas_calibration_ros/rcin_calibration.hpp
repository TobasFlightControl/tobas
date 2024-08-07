#pragma once

#include <tobas_property_tools/property_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_hal_msgs/Sbus.h>
#include <tobas_calibration_msgs/RCInputCalibration.h>

namespace tobas_calibration
{
class RCInputCalibrationRos : public tobas::BaseNode
{
  static constexpr char kServiceName[] = "rcin_calibration";

  static constexpr size_t kMinSignalRange = 300;

  using self = RCInputCalibrationRos;
  using super = tobas::BaseNode;
  using SrvType = tobas_calibration_msgs::RCInputCalibration;

public:
  explicit RCInputCalibrationRos(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  ptree::PropertyClient property_client_;
  rclcpp::ServiceServer ss_;

  bool executeCb(SrvType::Request& req, SrvType::Response& res);
};
}  // namespace tobas_calibration
