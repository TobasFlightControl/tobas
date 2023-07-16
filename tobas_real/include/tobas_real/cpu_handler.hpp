#pragma once

#include <ros/ros.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Cpu.h>

namespace tobas_real
{
class CpuHandler : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 10.;  // [Hz]
  static constexpr char kTemperatureFilePath[] = "/sys/class/thermal/thermal_zone0/temp";

  using super = tobas::BaseNode;

public:
  explicit CpuHandler();

  void run();

private:
  int temp_millidegrees_;
  tobas_msgs::Cpu cpu_msg_;

  ros::Publisher cpu_pub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::Event& event) override;
};
}  // namespace tobas_real
