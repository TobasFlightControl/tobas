#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

#include <navio2/ADC.h>
#include <tobas_tools/node.hpp>

namespace tobas_real
{
class BatteryHandler : public tobas::BaseNode
{
  static constexpr size_t kUpdateRate = 100;       // [Hz]
  static constexpr double kLpfTimeConst = 10.;     // [s]
  static constexpr double kVoltageThreshold = 3.;  // [V]

  using self = BatteryHandler;
  using super = tobas::BaseNode;

public:
  explicit BatteryHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  ADC adc_;

  // Config
  double adc_coef_;

  // Publisher
  ros::Publisher battery_pub_;

  // Timer
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void getAdcCoefficient();

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_real
