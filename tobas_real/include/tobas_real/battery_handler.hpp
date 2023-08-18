#pragma once

#include <ros/ros.h>
#include <Navio2/ADC_Navio2.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Battery.h>

namespace tobas_real
{
class BatteryHandler : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 100.;  // [Hz]

  using super = tobas::BaseNode;

public:
  explicit BatteryHandler();

  void run();

private:
  ADC_Navio2 adc_;
  double adc_coef_;
  tobas_msgs::Battery battery_msg_;

  // Publisher
  ros::Publisher battery_pub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void getAdcCoefficient();

  void eventCb(const tobas_msgs::Event& event) override;
};
}  // namespace tobas_real
