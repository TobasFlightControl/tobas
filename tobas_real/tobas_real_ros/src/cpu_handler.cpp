#include <fstream>

#include <tobas_std_tools/unix.hpp>
#include <tobas_std_tools/string.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Cpu.h>

#include "../include/tobas_real_ros/cpu_handler.hpp"

using namespace std;

namespace tobas_real_ros
{
CpuHandler::CpuHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  registerPublishers();
  registerSubscribers();

  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);
}

void CpuHandler::getRosParams()
{
}

void CpuHandler::registerPublishers()
{
  cpu_pub_ = nh_.advertise<tobas_msgs::Cpu>(tobas::kCpuTopic, 1);
}

void CpuHandler::registerSubscribers()
{
}

void CpuHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // Get CPU temperature
  ifstream file(kTemperatureFilePath);
  if (!file)
  {
    rosError(name_, "Failed to open " << kTemperatureFilePath << ".");
    return;
  }
  file >> temp_millidegrees_;

  // Get CPU frequency
  const auto vcgencmd_out = tobas_std::exec_command("vcgencmd measure_clock arm");
  const auto freq_str = tobas_std::deleteNl(tobas_std::split(vcgencmd_out, '=').back());
  const auto freq = stoul(freq_str);

  // Create ROS message
  const auto cpu_msg = boost::make_shared<tobas_msgs::Cpu>();
  cpu_msg->header.stamp = event.current_real;
  cpu_msg->temperature = static_cast<double>(temp_millidegrees_) * 1e-3;
  cpu_msg->frequency = freq;

  // Publish ROS message
  cpu_pub_.publish(cpu_msg);
}
}  // namespace tobas_real_ros
