#pragma once

#include <std_srvs/Trigger.h>

#include <tobas_math/ellipse_transformer.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_hal_msgs/MagneticField.h>

namespace tobas_real_ros
{
class MagnetometerHandler : public tobas::BaseNode
{
  static constexpr double kHpfCutoff = 10.;  // [Hz] (G(1Hz) ~ 0.1, G(20Hz) ~ 0.9)
  static constexpr size_t kWindowSize = 100;

  using self = MagnetometerHandler;
  using super = tobas::BaseNode;

public:
  explicit MagnetometerHandler(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  // Config
  math::EllipseTransformer mag_trans_;

  tobas_hal_msgs::MagneticFieldConstPtr mag_raw_;
  ptree::PropertyClient property_client_;
  std::array<dsp::NoiseVarianceFilter, 3> mag_noise_;

  rclcpp::Publisher mag_pub_;
  rclcpp::Subscriber mag_sub_;
  rclcpp::ServiceServer reload_config_srv_;

  bool reloadConfig();

  void magCb(const tobas_hal_msgs::MagneticFieldConstPtr& mag_raw);
  bool reloadConfigCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
};
}  // namespace tobas_real_ros
