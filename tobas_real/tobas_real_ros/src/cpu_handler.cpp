#include <fstream>
#include <sstream>

#include <tobas_std_tools/string.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Cpu.h>

#include "../include/tobas_real_ros/cpu_handler.hpp"

using namespace std;

namespace tobas_real_ros
{
CpuHandler::CpuHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  cpu_pub_ = nh_.advertise<tobas_msgs::Cpu>(tobas::kCpuTopic, 1);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);
}

bool CpuHandler::getTemperature(double& temp)
{
  ifstream temp_file(kTemperatureFilePath);
  if (!temp_file)
  {
    TOBAS_ERROR("Failed to open ", kTemperatureFilePath, ".");
    return false;
  }
  temp_file >> temp_millidegrees_;
  temp = static_cast<double>(temp_millidegrees_) * 1e-3;
  return true;
}

bool CpuHandler::getFrequency(uint64_t& freq)
{
  auto vcgencmd_out = linux::executeCommand("vcgencmd measure_clock arm");
  vcgencmd_out.pop_back();                                           // 改行コードを削除
  const auto freq_str = tobas_std::split(vcgencmd_out, '=').back();  // 数値部分のみ抜き出す
  freq = stoul(freq_str);                                            // str -> uint64
  return true;
}

bool CpuHandler::getLoad(double& load)
{
  // ファイルを読み込む
  ifstream stat_file(kStatisticsFilePath);
  if (!stat_file)
  {
    TOBAS_ERROR("Failed to open ", kStatisticsFilePath, ".");
    return false;
  }

  // ファイルの最初の行を読む
  if (!getline(stat_file, cpu_line_))
  {
    TOBAS_ERROR("Failed to read the first line of ", kStatisticsFilePath, ".");
    return false;
  }

  // CPUの行を空白で区切る
  istringstream iss(cpu_line_);

  // CPU使用時間を取得 (http://my-web-site.iobb.net/~yuki/2017-10/raspberry-pi/cpustat/)
  // 最初のトークン（"cpu"）を読み飛ばす
  iss >> token_;

  // (01) Time spent in user mode
  if (!(iss >> token_))
  {
    TOBAS_ERROR("Failed to read the CPU time spent in user mode.");
    return false;
  }
  const auto new_user_time = stoul(token_);

  // (02) Time spent in user mode with low priority (nice)
  if (!(iss >> token_))
  {
    TOBAS_ERROR("Failed to read the CPU time spent in user mode with low priority.");
    return false;
  }
  const auto new_nice_time = stoul(token_);

  // (03) Time spent in system mode
  if (!(iss >> token_))
  {
    TOBAS_ERROR("Failed to read the CPU time spent in system mode.");
    return false;
  }
  const auto new_system_time = stoul(token_);

  // (04) Time spent in the idle task
  if (!(iss >> token_))
  {
    TOBAS_ERROR("Failed to read the CPU time spent in the idle task.");
    return false;
  }
  const auto new_idle_time = stoul(token_);

  // 前回値との差分を計算
  const auto user_time = new_user_time - prev_user_time_;
  const auto nice_time = new_nice_time - prev_nice_time_;
  const auto system_time = new_system_time - prev_system_time_;
  const auto idle_time = new_idle_time - prev_idle_time_;

  // 負荷を計算
  const auto busy_time = user_time + nice_time + system_time;
  const auto all_time = busy_time + idle_time;
  load = static_cast<double>(busy_time) / static_cast<double>(all_time);

  // CPU使用時間を更新
  prev_user_time_ = new_user_time;
  prev_nice_time_ = new_nice_time;
  prev_system_time_ = new_system_time;
  prev_idle_time_ = new_idle_time;

  return true;
}

void CpuHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // Create ROS message
  const auto cpu_msg = boost::make_shared<tobas_msgs::Cpu>();
  cpu_msg->header.stamp = event.current_real;

  // Get CPU temperature
  if (!getTemperature(cpu_msg->temperature))
    return;

  // Get CPU frequency
  if (!getFrequency(cpu_msg->frequency))
    return;

  // Get CPU load
  if (!getLoad(cpu_msg->load))
    return;

  // Publish ROS message
  cpu_pub_.publish(cpu_msg);
}
}  // namespace tobas_real_ros
