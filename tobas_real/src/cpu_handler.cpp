#include <fstream>

#include <tobas_ros_tools/console_message.hpp>

#include <tobas_msgs/Cpu.h>

#include "../include/tobas_real/cpu_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
CpuHandler::CpuHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  registerPublishers();
  registerSubscribers();

  main_timer_ = nh_.createTimer(kUpdateRate, &self::mainTimerCb, this);
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
  super::registerSubscribers();
}

void CpuHandler::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      main_timer_.stop();
      break;
    default:
      break;
  }
}

void CpuHandler::mainTimerCb(const ros::TimerEvent& event)
{
  ifstream file(kTemperatureFilePath);
  if (!file)
  {
    rosErrorThrottle(kErrorPeriod, name_, "Failed to open " << kTemperatureFilePath << ".");
    return;
  }
  file >> temp_millidegrees_;

  const auto cpu_msg = boost::make_shared<tobas_msgs::Cpu>();
  cpu_msg->header.stamp = event.current_real;
  cpu_msg->temperature = static_cast<double>(temp_millidegrees_) * 1e-3;

  cpu_pub_.publish(cpu_msg);
}
}  // namespace tobas_real
