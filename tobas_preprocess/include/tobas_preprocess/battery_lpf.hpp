#pragma once

#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_msgs/Battery.h>

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
  explicit BatteryLpf(, const std::string& name = rclcpp::this_node::getName());

private:
  dsp::LowPassFilter<double> voltage_lpf_, current_lpf_;
  tobas_msgs::BatteryConstPtr last_msg_;

  rclcpp::Publisher battery_lpf_pub_;
  rclcpp::Subscriber battery_raw_sub_;

  void batteryRawCb(const tobas_msgs::BatteryConstPtr& battery_raw);
};
}  // namespace tobas_preprocess
