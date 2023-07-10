#pragma once

#include <ros/ros.h>
#include <Navio2/ADC_Navio2.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Battery.h>

namespace tobas_real
{
class BatteryHandler : public tobas::BaseNode
{
  using super = tobas::BaseNode;

public:
  explicit BatteryHandler();

  void run();

private:
  ADC_Navio2 adc_;
  tobas_msgs::Battery battery_msg_;

  // Publisher
  ros::Publisher battery_pub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::Event& event) override;
};
}  // namespace tobas_real
