#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

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
  explicit CpuHandler(ros::NodeHandle nh, ros::NodeHandle pnh);

private:
  int temp_millidegrees_;
  tobas_msgs::Cpu cpu_msg_;

  // Publisher
  ros::Publisher cpu_pub_;

  // Timer
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::Event& event) override;
  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_real
