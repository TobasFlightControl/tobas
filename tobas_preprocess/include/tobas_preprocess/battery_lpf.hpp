#pragma once

#include <ros/ros.h>

#include <tobas_std_tools/first_order_filter.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Battery.h>

namespace tobas_preprocess
{
class BatteryLpf : public tobas::BaseNode
{
  static constexpr double kDefaultLpfTimeConst = 0.1;  // [s]

  using self = BatteryLpf;
  using super = tobas::BaseNode;

public:
  explicit BatteryLpf(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  // rosparams
  double lpf_time_const_;

  tobas_std::FirstOrderFilter<double> lpf_;
  ros::Time t_last_;

  ros::Publisher battery_lpf_pub_;
  ros::Subscriber battery_raw_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void batteryRawCb(const tobas_msgs::BatteryConstPtr& battery_raw);
};
}  // namespace tobas_preprocess
