#include <tobas_ros_tools/console_message.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_msgs/Throttles.h>

#include "../include/tobas_gazebo/rotor_command_handler.hpp"

using namespace std;

namespace tobas_gazebo
{
RotorCommandHandler::RotorCommandHandler(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  registerPublishers();
  registerSubscribers();
}

void RotorCommandHandler::getRosParams()
{
}

void RotorCommandHandler::registerPublishers()
{
  throttles_pub_ = nh_.advertise<tobas_msgs::Throttles>(tobas::kThrottlesCmdTopic, 1);
}

void RotorCommandHandler::registerSubscribers()
{
  battery_sub_ = nh_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this);
  tar_speeds_sub_ = nh_.subscribe(tobas::kRotorSpeedsCmdTopic, 1, &self::targetRotorSpeedsCb, this);
}

void RotorCommandHandler::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void RotorCommandHandler::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void RotorCommandHandler::targetRotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& tar_speeds)
{
  if (battery_ == nullptr)
    return;

  const auto data_size = tar_speeds->speeds.size();

  // Create throttle message
  const auto throttles = boost::make_shared<tobas_msgs::Throttles>();
  throttles->header = tar_speeds->header;
  throttles->data.resize(data_size);

  for (size_t rotor_idx = 0; rotor_idx < data_size; ++rotor_idx)
  {
    // Check the validity of the target rotation speed
    const auto tar_speed = tobas::clampTargetRotSpeedAndWarn(
      drone_, rotor_idx, battery_->voltage, tar_speeds->speeds[rotor_idx]);

    // Fill throttle
    throttles->data[rotor_idx] =
      drone_.throttleFromRotSpeed(rotor_idx, tar_speed, battery_->voltage);
  }

  // Publish throttle message
  throttles_pub_.publish(throttles);
}
}  // namespace tobas_gazebo
