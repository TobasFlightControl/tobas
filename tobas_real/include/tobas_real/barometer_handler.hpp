#pragma once

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <Common/MS5611.h>

#include <tobas_tools/node.hpp>

namespace tobas_real
{
class BarometerHandler : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 50.;  // [Hz]

  using super = tobas::BaseNode;

  using BarMsg = sensor_msgs::FluidPressure;

public:
  explicit BarometerHandler();

  void run();

private:
  MS5611 barometer_;
  BarMsg bar_msg_;

  // Config
  double pressure_noise_density_;  // [Pa/sqrt(Hz)]

  // PubSub
  ros::Publisher bar_pub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void readConfig();

  void eventCb(const tobas_msgs::Event& event) override;
};
}  // namespace tobas_real
