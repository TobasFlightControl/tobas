#include <fstream>

#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../include/tobas_real/cpu_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
constexpr char CpuHandler::kTemperatureFilePath[];

CpuHandler::CpuHandler() : super()
{
  getRosParams();
  registerPublishers();
  registerSubscribers();
}

void CpuHandler::run()
{
  dh_ros::Rate rate(kUpdateRate);

  while (ros::ok())
  {
    ifstream file(kTemperatureFilePath);
    if (!file)
    {
      rosErrorThrottle(kErrorPeriod, "Failed to open " << kTemperatureFilePath << ".");
    }
    file >> temp_millidegrees_;

    cpu_msg_.header.stamp = ros::Time::now();
    cpu_msg_.temperature = static_cast<double>(temp_millidegrees_) * 1e-3;

    cpu_pub_.publish(cpu_msg_);

    ros::spinOnce();
    rate.sleep();
  }
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

void CpuHandler::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      // ros::shutdown();
      break;
    default:
      break;
  }
}
}  // namespace tobas_real
