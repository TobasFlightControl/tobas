// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <fstream>
#include <sstream>

#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_linux/command_executor.hpp>
#include <tobas_node/node.hpp>
#include <tobas_string_tools/core.hpp>

#include <tobas_msgs/msg/cpu.hpp>

using namespace std::chrono_literals;
namespace ch = std::chrono;

namespace tobas
{
namespace real
{
class CpuHandlerNode : public BaseNode
{
  static constexpr auto kSamplingPeriod = 100ms;
  static constexpr char kTemperatureFilePath[] = "/sys/class/thermal/thermal_zone0/temp";
  static constexpr char kStatisticsFilePath[] = "/proc/stat";

  using self = CpuHandlerNode;
  using super = BaseNode;

public:
  explicit CpuHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  int temp_millidegrees_;
  std::string cpu_line_, token_;
  uint64_t prev_user_time_ = 0, prev_nice_time_ = 0, prev_system_time_ = 0, prev_idle_time_ = 0;
  uint64_t freq_;
  double temp_, load_;
  dsp::LowPassFilter<double> temp_lpf_, load_lpf_;
  linux::CommandExecutor command_executor_;

  // Publisher
  ros2::PublisherPtr<tobas_msgs::msg::Cpu> cpu_pub_;

  // Timer
  ros2::TimerPtr main_timer_;

  bool getFrequency();
  bool getTemperature();
  bool getLoad();

  bool getStatus();

  void mainTimerCb();
};

CpuHandlerNode::CpuHandlerNode(const rclcpp::NodeOptions& options)
  : super("real_cpu_handler", nodeOptions_Default(options))
{
  // Configure the LPFs.
  temp_lpf_.setCutoffFrequency(dsp::cutoffFromTimeConst(5.0));
  load_lpf_.setCutoffFrequency(dsp::cutoffFromTimeConst(1.0));

  // Set the initial status.
  if (!getStatus()) {
    TOBAS_ERROR("This node will not work.");
    return;
  }
  temp_lpf_.setValue(temp_);
  load_lpf_.setValue(load_);

  // Resister the ROS interfaces.
  cpu_pub_ = createPublisher<tobas_msgs::msg::Cpu>(topic::kCpu);
  main_timer_ = createWallTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

bool CpuHandlerNode::getFrequency()
{
  if (!command_executor_.execute("vcgencmd measure_clock arm")) {
    TOBAS_ERROR("Failed to get CPU clock frequency.");
    return false;
  }

  const auto freq_str = str::split(command_executor_.getOutput(), '=').back();  // Extract only the numeric part.
  freq_ = stoul(freq_str);                                                      // str -> uint64
  return true;
}

bool CpuHandlerNode::getTemperature()
{
  std::ifstream temp_file(kTemperatureFilePath);
  if (!temp_file) {
    TOBAS_ERROR("Failed to open ", kTemperatureFilePath, ".");
    return false;
  }
  temp_file >> temp_millidegrees_;
  temp_ = static_cast<double>(temp_millidegrees_) * 1e-3;
  return true;
}

bool CpuHandlerNode::getLoad()
{
  // Load the file.
  std::ifstream stat_file(kStatisticsFilePath);
  if (!stat_file) {
    TOBAS_ERROR("Failed to open ", kStatisticsFilePath, ".");
    return false;
  }

  // Read the first line of the file.
  if (!std::getline(stat_file, cpu_line_)) {
    TOBAS_ERROR("Failed to read the first line of ", kStatisticsFilePath, ".");
    return false;
  }

  // Split the CPU line by whitespace.
  std::istringstream iss(cpu_line_);

  // Get CPU usage times.
  // Skip the first token, "cpu".
  iss >> token_;

  // 1. Time spent in user mode
  if (!(iss >> token_)) {
    TOBAS_ERROR("Failed to read the CPU time spent in user mode.");
    return false;
  }
  const auto new_user_time = stoul(token_);

  // 2. Time spent in user mode with low priority (nice)
  if (!(iss >> token_)) {
    TOBAS_ERROR("Failed to read the CPU time spent in user mode with low priority.");
    return false;
  }
  const auto new_nice_time = stoul(token_);

  // 3. Time spent in system mode
  if (!(iss >> token_)) {
    TOBAS_ERROR("Failed to read the CPU time spent in system mode.");
    return false;
  }
  const auto new_system_time = stoul(token_);

  // 4. Time spent in the idle task
  if (!(iss >> token_)) {
    TOBAS_ERROR("Failed to read the CPU time spent in the idle task.");
    return false;
  }
  const auto new_idle_time = stoul(token_);

  // Calculate differences from previous values.
  const auto user_time = new_user_time - prev_user_time_;
  const auto nice_time = new_nice_time - prev_nice_time_;
  const auto system_time = new_system_time - prev_system_time_;
  const auto idle_time = new_idle_time - prev_idle_time_;

  // Calculate load.
  const auto busy_time = user_time + nice_time + system_time;
  const auto all_time = busy_time + idle_time;
  load_ = static_cast<double>(busy_time) / static_cast<double>(all_time);

  // Update CPU usage times.
  prev_user_time_ = new_user_time;
  prev_nice_time_ = new_nice_time;
  prev_system_time_ = new_system_time;
  prev_idle_time_ = new_idle_time;

  return true;
}

bool CpuHandlerNode::getStatus()
{
  if (!getFrequency()) {
    return false;
  }

  if (!getTemperature()) {
    return false;
  }

  if (!getLoad()) {
    return false;
  }

  return true;
}

void CpuHandlerNode::mainTimerCb()
{
  // Get CPU status.
  if (!getStatus()) {
    return;
  }

  // Smooth CPU load.
  constexpr auto dt = ch::duration<double>(kSamplingPeriod).count();
  temp_lpf_.update(temp_, dt);
  load_lpf_.update(load_, dt);

  // Publish CPU status.
  auto cpu_msg = std::make_unique<tobas_msgs::msg::Cpu>();
  cpu_msg->header.stamp = now();
  cpu_msg->frequency = freq_;
  cpu_msg->temperature = temp_lpf_.getValue();
  cpu_msg->load = load_lpf_.getValue();
  cpu_pub_->publish(std::move(cpu_msg));
}
}  // namespace real
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::real::CpuHandlerNode)
