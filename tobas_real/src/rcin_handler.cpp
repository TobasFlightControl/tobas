#include <boost/property_tree/ini_parser.hpp>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/vector.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_msgs/RCInput.h>

#include "../include/tobas_real/rcin_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_real
{
RCInputHandler::RCInputHandler(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name)
{
  getRosParams();

  readConfig();
  rcin_.initialize();

  registerPublishers();
  registerSubscribers();

  main_timer_ = nh_.createTimer(kUpdateRate, &RCInputHandler::mainTimerCb, this);
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
  event_sub_ = nh_.subscribe("event", 1, &RCInputHandler::eventCb, this, tcpNoDelay());
}

void RCInputHandler::readConfig()
{
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini(kConfigPath, pt);

  roll_range_.lower = pt.get<double>(kConfigKey_RcRollLeft);
  roll_range_.upper = pt.get<double>(kConfigKey_RcRollRight);

  pitch_range_.lower = pt.get<double>(kConfigKey_RcPitchDown);
  pitch_range_.upper = pt.get<double>(kConfigKey_RcPitchUp);

  yaw_range_.lower = pt.get<double>(kConfigKey_RcYawRight);
  yaw_range_.upper = pt.get<double>(kConfigKey_RcYawLeft);

  thrust_range_.lower = pt.get<double>(kConfigKey_RcThrustDown);
  thrust_range_.upper = pt.get<double>(kConfigKey_RcThrustUp);

  estop_range_.lower = pt.get<double>(kConfigKey_RcEStopDown);
  estop_range_.upper = pt.get<double>(kConfigKey_RcEStopUp);

  const auto num_modes = pt.get<uint32_t>(kConfigKey_RcNrOfModes);
  modes_.resize(num_modes);
  for (uint32_t i = 0; i < num_modes; ++i)
  {
    const string key = kConfigKey_RcModePrefix + to_string(i);
    modes_[i] = pt.get<double>(key);
  }
}

void RCInputHandler::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
      break;
    default:
      break;
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

  // Create message
  const auto rcin_msg = boost::make_shared<tobas_msgs::RCInput>();
  rcin_msg->header.stamp = event.current_real;
  rcin_msg->roll = remap<double>(roll_period, roll_range_.lower, roll_range_.upper, -1, 1);
  rcin_msg->pitch = remap<double>(pitch_period, pitch_range_.lower, pitch_range_.upper, -1, 1);
  rcin_msg->yaw = remap<double>(yaw_period, yaw_range_.lower, yaw_range_.upper, -1, 1);
  rcin_msg->thrust = remap<double>(thrust_period, thrust_range_.lower, thrust_range_.upper, 0, 1);
  rcin_msg->e_stop = estop_period < estop_range_.mean();
  rcin_msg->mode = dh_std::closestIndex<double>(modes_, mode_period);

  // Publish message
  rcin_pub_.publish(rcin_msg);
}
}  // namespace tobas_real
