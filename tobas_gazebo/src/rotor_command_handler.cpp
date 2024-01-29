#include <tobas_ros_tools/console_message.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_msgs/Throttles.h>

#include "../include/tobas_gazebo/rotor_command_handler.hpp"
#include "../include/tobas_gazebo/constants.hpp"

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

  arm_rotors_ss_ = nh_.advertiseService(tobas::kArmRotorsSrv, &self::armRotorsCb, this);
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

void RotorCommandHandler::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void RotorCommandHandler::targetRotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& tar_speeds)
{
  if (battery_ == nullptr)
  {
    rosErrorThrottle(
      kErrorPeriod, name_,
      "The rotors cannot be rotated because battery state has not been received yet.");
    return;
  }

  if (!is_armed_)
  {
    rosErrorThrottle(
      kErrorPeriod, name_, "The rotors cannot be rotated because they are disarmed.");
    return;
  }

  const auto data_size = tar_speeds->speeds.size();
  if (data_size != drone_.numRotors())
  {
    rosError(name_, "Size mismatch: " << data_size << " != " << drone_.numRotors());
    return;
  }

  // Create throttle message
  const auto throttles = boost::make_shared<tobas_msgs::Throttles>();
  throttles->header = tar_speeds->header;
  throttles->data.resize(data_size);

  for (size_t rotor_idx = 0; rotor_idx < data_size; ++rotor_idx)
  {
    // Check the validity of the target rotation speed
    const auto min_speed = drone_.minRotSpeed(rotor_idx, battery_->voltage);
    const auto max_speed = drone_.maxRotSpeed(rotor_idx, battery_->voltage);
    auto tar_speed = tar_speeds->speeds[rotor_idx];
    if (tar_speed < min_speed - tobas::kRotSpeedMargin)
    {
      ROS_WARN_STREAM(
        "Target rotation speed of CH" << rotor_idx << " is too low: " << tar_speed << " < "
                                      << min_speed << " [rad/s]");
      tar_speed = min_speed;
    }
    else if (tar_speed > max_speed + tobas::kRotSpeedMargin)
    {
      ROS_WARN_STREAM(
        "Target rotation speed of CH" << rotor_idx << " is too high: " << tar_speed << " > "
                                      << max_speed << " [rad/s]");
      tar_speed = max_speed;
    }

    // Fill throttle
    throttles->data[rotor_idx] =
      drone_.throttleFromRotSpeed(rotor_idx, tar_speed, battery_->voltage);
  }

  // Publish throttle message
  throttles_pub_.publish(throttles);
}

bool RotorCommandHandler::armRotorsCb(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
{
  if (!is_armed_ && req.data)
  {
    rosInfo(name_, "Arming rotors.");
  }
  else if (is_armed_ && !req.data)
  {
    rosInfo(name_, "Disarming rotors.");
  }

  is_armed_ = req.data;
  res.success = true;
  return true;
}
}  // namespace tobas_gazebo
