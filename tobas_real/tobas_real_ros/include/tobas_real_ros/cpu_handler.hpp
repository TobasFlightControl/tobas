#pragma once

#include <rclcpp/rclcpp.hpp>
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
  explicit CpuHandler(, const std::string& name = rclcpp::this_node::getName());

private:
  int temp_millidegrees_;
  std::string cpu_line_, token_;
  uint64_t prev_user_time_ = 0, prev_nice_time_ = 0, prev_system_time_ = 0, prev_idle_time_ = 0;

  // Publisher
  rclcpp::Publisher cpu_pub_;

  // Timer
  rclcpp::Timer main_timer_;

  bool getTemperature(double& temp);
  bool getFrequency(uint64_t& freq);
  bool getLoad(double& load);

  void mainTimerCb(const rclcpp::TimerEvent& event);
};
}  // namespace tobas_real_ros
