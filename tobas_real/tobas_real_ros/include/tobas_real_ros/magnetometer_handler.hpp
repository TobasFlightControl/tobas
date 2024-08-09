#pragma once

#include <std_srvs/srv/trigger.hpp>

#include <tobas_math/ellipse_transformer.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_node/node.hpp>
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
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Config
  math::EllipseTransformer mag_trans_;

  tobas_hal_msgs::MagneticField::ConstSharedPtr mag_raw_;
  ptree::PropertyClient property_client_;
  std::array<dsp::NoiseVarianceFilter, 3> mag_noise_;

  PublisherPtr<> mag_pub_;
  SubscriberPtr<> mag_sub_;
  rclcpp::ServiceServer reload_config_srv_;

  bool reloadConfig();

  void magCb(const tobas_hal_msgs::MagneticField::ConstSharedPtr& mag_raw);
  bool reloadConfigCb(std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res);
};
}  // namespace tobas_real_ros
