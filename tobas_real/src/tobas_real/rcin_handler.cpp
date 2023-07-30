#include <boost/property_tree/ini_parser.hpp>

#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/rate.hpp>

#include "../../include/tobas_real/rcin_handler.hpp"
#include "../../include/tobas_real/common.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_real
{
RCInputHandler::RCInputHandler()
{
  getRosParams();

  getRcPeriodRanges();
  rcin_.initialize();

  registerPublishers();
  registerSubscribers();
}

void RCInputHandler::run()
{
  dh_ros::Rate rate(kUpdateRate);

  while (ros::ok())
  {
    rcin_msg_.header.stamp = ros::Time::now();

    // Read RC input periods
    const auto roll_period = rcin_.read(kRCInputChannelRoll);
    const auto pitch_period = rcin_.read(kRCInputChannelPitch);
    const auto yaw_period = rcin_.read(kRCInputChannelYaw);
    const auto throttle_period = rcin_.read(kRCInputChannelThrottle);
    const auto toggle_period = rcin_.read(kRCInputChannelToggle);

    // Fill message
    // TODO: Consider neutoral positions
    rcin_msg_.roll = remap<double>(roll_period, roll_range_.lower, roll_range_.upper, -1, 1);
    rcin_msg_.pitch = -remap<double>(pitch_period, pitch_range_.lower, pitch_range_.upper, -1, 1);
    rcin_msg_.yaw = -remap<double>(yaw_period, yaw_range_.lower, yaw_range_.upper, -1, 1);
    rcin_msg_.throttle =
      remap<double>(throttle_period, throttle_range_.lower, throttle_range_.upper, 0, 1);
    rcin_msg_.toggle = toggle_period < toggle_range_.mean();

    // Publish message
    rcin_pub_.publish(rcin_msg_);

    ros::spinOnce();
    rate.sleep();
  }
}

void RCInputHandler::getRosParams()
{
}

void RCInputHandler::registerPublishers()
{
  rcin_pub_ = nh_.advertise<tobas_msgs::RCInput>("rc_input", 1);
}

void RCInputHandler::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &RCInputHandler::eventCb, this);
}

void RCInputHandler::getRcPeriodRanges()
{
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini(kConfigPath, pt);

  roll_range_.lower = pt.get<double>(kConfigKey_RcRollLeft);
  roll_range_.upper = pt.get<double>(kConfigKey_RcRollRight);

  pitch_range_.lower = pt.get<double>(kConfigKey_RcPitchUp);
  pitch_range_.upper = pt.get<double>(kConfigKey_RcPitchDown);

  yaw_range_.lower = pt.get<double>(kConfigKey_RcYawLeft);
  yaw_range_.upper = pt.get<double>(kConfigKey_RcYawRight);

  throttle_range_.lower = pt.get<double>(kConfigKey_RcThrottleDown);
  throttle_range_.upper = pt.get<double>(kConfigKey_RcThrottleUp);

  toggle_range_.lower = pt.get<double>(kConfigKey_RcToggleUp);
  toggle_range_.upper = pt.get<double>(kConfigKey_RcToggleDown);
}

void RCInputHandler::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      ros::shutdown();
      break;
    default:
      break;
  }
}
}  // namespace tobas_real
