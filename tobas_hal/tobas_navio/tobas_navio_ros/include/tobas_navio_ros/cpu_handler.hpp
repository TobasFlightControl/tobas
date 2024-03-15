#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

#include <tobas_tools/node.hpp>

namespace tobas_navio_ros
{
class CpuHandler : public tobas::BaseNode
{
  static constexpr size_t kUpdateRate = 10;  // [Hz]
  static constexpr char kTemperatureFilePath[] = "/sys/class/thermal/thermal_zone0/temp";

  using self = CpuHandler;
  using super = tobas::BaseNode;

public:
  explicit CpuHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  int temp_millidegrees_;

  // Publisher
  ros::Publisher cpu_pub_;

  // Timer
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros
