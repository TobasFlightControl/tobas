#include <fstream>

#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_msgs/Cpu.h>

#include "../include/tobas_real/cpu_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
constexpr char CpuHandler::kTemperatureFilePath[];

CpuHandler::CpuHandler(ros::NodeHandle nh, ros::NodeHandle pnh, string name) : super(nh, pnh, name)
{
  getRosParams();
  registerPublishers();
  registerSubscribers();

  main_timer_ = nh_.createTimer(ros::Duration(1 / kUpdateRate), &CpuHandler::mainTimerCb, this);
}

void CpuHandler::getRosParams()
{
}

void CpuHandler::registerPublishers()
{
  cpu_pub_ = nh_.advertise<tobas_msgs::Cpu>("cpu", 1);
}

void CpuHandler::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &CpuHandler::eventCb, this);
}

void CpuHandler::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      // nh_.shutdown();
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
