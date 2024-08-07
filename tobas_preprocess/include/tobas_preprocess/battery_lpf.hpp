#pragma once

#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_node/node.hpp>
#include <tobas_msgs/msg/battery.hpp>

namespace tobas_preprocess
{
class BatteryLpf : public tobas::BaseNode
{
  // LPFのカットオフ周波数 [Hz]
  // 小さすぎると離陸時の急激な電圧降下に追従できず，所望の推力が出ない．
  // ADCのノイズを軽減できる最大限の値に設定すべき．
  static constexpr double kLpfCutoff = 1.;

  using self = BatteryLpf;
  using super = tobas::BaseNode;

public:
  explicit BatteryLpf(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  dsp::LowPassFilter<double> voltage_lpf_, current_lpf_;
  tobas_msgs::msg::Battery::ConstSharedPtr last_msg_;

  PublisherPtr<> battery_lpf_pub_;
  SubscriberPtr<> battery_raw_sub_;

  void batteryRawCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery_raw);
};
}  // namespace tobas_preprocess
