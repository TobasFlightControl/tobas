#pragma once

#include <ros/ros.h>

#include <tobas_std_tools/first_order_filter.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Battery.h>

namespace tobas_preprocess
{
class BatteryLpf : public tobas::BaseNode
{
  // LPFの時定数 [s]
  // 大きくしすぎると離陸時の急激な電圧降下に追従できず，所望の推力が出ない．
  // ADCのノイズを軽減できる最小限の時定数に設定すべき．
  static constexpr double kLpfTimeConst = 0.1;

  using self = BatteryLpf;
  using super = tobas::BaseNode;

public:
  explicit BatteryLpf(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas_std::FirstOrderFilter<double> voltage_lpf_;
  tobas_std::FirstOrderFilter<double> current_lpf_;
  ros::Time t_last_;

  ros::Publisher battery_lpf_pub_;
  ros::Subscriber battery_raw_sub_;

  void batteryRawCb(const tobas_msgs::BatteryConstPtr& battery_raw);
};
}  // namespace tobas_preprocess
