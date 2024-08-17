#pragma once

#include <tobas_property_tools/property_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_calibration_msgs/MagCalibration.h>

namespace tobas_calibration
{
class MagCalibrationRos : public tobas::BaseNode
{
  static constexpr char kServiceName[] = "mag_calibration";

  using self = MagCalibrationRos;
  using super = tobas::BaseNode;
  using SrvType = tobas_calibration_msgs::MagCalibration;

public:
  explicit MagCalibrationRos(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ptree::PropertyClient property_client_;
  ServicePtr<> ss_;

  bool isValidEllipse(const SrvType::Request& req);
  bool executeCb(SrvType::Request& req, SrvType::Response& res);
};
}  // namespace tobas_calibration
