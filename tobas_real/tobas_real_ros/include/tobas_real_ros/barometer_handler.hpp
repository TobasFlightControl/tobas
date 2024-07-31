#pragma once

#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_hal_msgs/FluidPressure.h>

namespace tobas_real_ros
{
class BarometerHandler : public tobas::BaseNode
{
  static constexpr double kHpfCutoff = 10.;  // [Hz] (G(1Hz) ~ 0.1, G(20Hz) ~ 0.9)
  static constexpr size_t kWindowSize = 100;

  static constexpr double kMinAirPressure = 30000.;   // [Pa] 有効な気圧の下限 (エベレスト山頂)
  static constexpr double kMaxAirPressure = 120000.;  // [Pa] 有効な気圧の上限 (観測史上最大以上)

  using self = BarometerHandler;
  using super = tobas::BaseNode;

public:
  explicit BarometerHandler(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  tobas_hal_msgs::FluidPressureConstPtr bar_raw_;
  dsp::NoiseVarianceFilter pressure_noise_;

  rclcpp::Publisher bar_pub_;
  rclcpp::Subscriber bar_sub_;

  void airPressureCb(const tobas_hal_msgs::FluidPressureConstPtr& bar_raw);
};
}  // namespace tobas_real_ros
