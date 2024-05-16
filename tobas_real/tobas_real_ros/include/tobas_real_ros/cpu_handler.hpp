#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

#include <tobas_tools/node.hpp>

namespace tobas_real_ros
{
class CpuHandler : public tobas::BaseNode
{
  static constexpr size_t kSamplingRate = 1;  // [Hz]
  static constexpr char kTemperatureFilePath[] = "/sys/class/thermal/thermal_zone0/temp";
  static constexpr char kStatisticsFilePath[] = "/proc/stat";

  using self = CpuHandler;
  using super = tobas::BaseNode;

public:
  explicit CpuHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  int temp_millidegrees_;
  std::string cpu_line_, token_;
  uint64_t prev_user_time_ = 0, prev_nice_time_ = 0, prev_system_time_ = 0, prev_idle_time_ = 0;

  // Publisher
  ros::Publisher cpu_pub_;

  // Timer
  ros::Timer main_timer_;

  bool getTemperature(double& temp);
  bool getFrequency(uint64_t& freq);
  bool getLoad(double& load);

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_real_ros
