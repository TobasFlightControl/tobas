#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <Navio2/ADC_Navio2.h>

#include <dh_std_tools/first_order_filter.hpp>

#include <tobas_tools/node.hpp>

namespace tobas_real
{
class BatteryHandler : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 100.;      // [Hz]
  static constexpr double kLpfCutoffFreq = 1.;     // [Hz]
  static constexpr double kVoltageThreshold = 3.;  // [V]

  using super = tobas::BaseNode;

public:
  explicit BatteryHandler(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  ADC_Navio2 adc_;
  dh_std::FirstOrderFilter<double> lpf_;

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

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_real
