#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/property_tree.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/RCInput.h>

#include "../include/tobas_real/rcin_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;
using namespace tobas_std;

namespace tobas_real
{
RCInputHandler::RCInputHandler(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  readConfig();

  if (rcin_.initialize() < 0)
    ROS_THROW_NAMED(name_, "Failed to initialize RC input driver.");

  registerPublishers();
  registerSubscribers();

  main_timer_ = nh_.createTimer(kUpdateRate, &RCInputHandler::mainTimerCb, this);
}

void RCInputHandler::getRosParams()
{
}

void RCInputHandler::registerPublishers()
{
  rcin_pub_ = nh_.advertise<tobas_msgs::RCInput>(tobas::kRcInputTopic, 1);
}

void RCInputHandler::registerSubscribers()
{
}

void RCInputHandler::readConfig()
{
  tobas_std::PropertyTree pt(kConfigPath);

  pt.get(kConfigKey_RcRollLeft, roll_range_.lower);
  pt.get(kConfigKey_RcRollRight, roll_range_.upper);

  pt.get(kConfigKey_RcPitchDown, pitch_range_.lower);
  pt.get(kConfigKey_RcPitchUp, pitch_range_.upper);

  pt.get(kConfigKey_RcYawRight, yaw_range_.lower);
  pt.get(kConfigKey_RcYawLeft, yaw_range_.upper);

  pt.get(kConfigKey_RcThrustDown, thrust_range_.lower);
  pt.get(kConfigKey_RcThrustUp, thrust_range_.upper);

  pt.get(kConfigKey_RcEStopOn, estop_on_);
  pt.get(kConfigKey_RcEStopOff, estop_off_);

  pt.get(kConfigKey_RcGPSwOn, gpsw_on_);
  pt.get(kConfigKey_RcGPSwOff, gpsw_off_);

  pt.get(kConfigKey_RcNrOfModes, num_modes_);
  modes_.resize(num_modes_);
  for (size_t i = 0; i < num_modes_; ++i)
  {
    const string key = kConfigKey_RcModePrefix + to_string(i);
    pt.get(key, modes_[i]);
  }
}

void RCInputHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // Read RC input periods
  const auto roll_period = rcin_.read(kRcChannelRoll);
  const auto pitch_period = rcin_.read(kRcChannelPitch);
  const auto yaw_period = rcin_.read(kRcChannelYaw);
  const auto thrust_period = rcin_.read(kRcChannelThrust);
  const auto estop_period = rcin_.read(kRcChannelEStop);
  const auto mode_period = rcin_.read(kRcChannelMode);
  const auto gpsw_period = rcin_.read(kRcChannelGPSw);

  // Create message
  const auto rcin_msg = boost::make_shared<tobas_msgs::RCInput>();
  rcin_msg->header.stamp = event.current_real;
  rcin_msg->roll = remap<double>(roll_period, roll_range_.lower, roll_range_.upper, -1, 1);
  rcin_msg->pitch = remap<double>(pitch_period, pitch_range_.lower, pitch_range_.upper, -1, 1);
  rcin_msg->yaw = remap<double>(yaw_period, yaw_range_.lower, yaw_range_.upper, -1, 1);
  rcin_msg->thrust = remap<double>(thrust_period, thrust_range_.lower, thrust_range_.upper, 0, 1);
  rcin_msg->mode = tobas_std::closestIndex<double>(modes_, mode_period);
  rcin_msg->e_stop = abs(estop_period - estop_on_) < abs(estop_period - estop_off_);
  rcin_msg->gpsw = abs(gpsw_period - gpsw_on_) < abs(gpsw_period - gpsw_off_);

  // Check signal validity
  constexpr double kOnePlusMargin = 1 + kSignalMargin;
  if (
    rcin_msg->roll < -kOnePlusMargin || kOnePlusMargin < rcin_msg->roll
    || rcin_msg->pitch < -kOnePlusMargin || kOnePlusMargin < rcin_msg->pitch
    || rcin_msg->yaw < -kOnePlusMargin || kOnePlusMargin < rcin_msg->yaw
    || rcin_msg->thrust < -kSignalMargin || kOnePlusMargin < rcin_msg->thrust)
  {
    rosErrorThrottle(
      kErrorPeriod, name_,
      "The value of the RC input is invalid. "
      "Please check the connection between the transmitter and receiver "
      "and ensure that they are properly calibrated.");
    return;
  }

  // Publish message
  rcin_pub_.publish(rcin_msg);
}
}  // namespace tobas_real
