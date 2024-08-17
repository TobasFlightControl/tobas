#pragma once

#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_node/node.hpp>
#include <tobas_hal_msgs/msg/fluid_pressure.hpp>

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
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_hal_msgs::msg::FluidPressure::ConstSharedPtr bar_raw_;
  dsp::NoiseVarianceFilter pressure_noise_;

  PublisherPtr<> bar_pub_;
  SubscriberPtr<> bar_sub_;

  void airPressureCb(const tobas_hal_msgs::msg::FluidPressure::ConstSharedPtr& bar_raw);
};
}  // namespace tobas_real_ros
