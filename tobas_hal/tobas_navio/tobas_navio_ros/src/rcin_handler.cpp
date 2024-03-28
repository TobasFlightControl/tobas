#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/array.hpp>
#include <tobas_std_tools/property_tree.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_msgs/RCInput.h>

#include "../include/tobas_navio_ros/rcin_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;
using namespace tobas_std;

namespace tobas_navio_ros
{
RCInputHandler::RCInputHandler(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  reloadConfig();

  if (rcin_.initialize() != navio::RCInput::E_NO_ERROR)
    ROS_EXIT_NAMED(nh_, name_, "Failed to initialize RC input driver.");

  registerPublishers();
  registerSubscribers();

  reload_config_srv_ =
    nh_.advertiseService(name + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);
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

void RCInputHandler::reloadConfig()
{
  tobas_std::PropertyTree pt(kConfigPath);

  pt.get(kConfigKey_RcRollLeft, roll_range_.lower, tobas_navio_ros::kPwmMin);
  pt.get(kConfigKey_RcRollRight, roll_range_.upper, tobas_navio_ros::kPwmMax);

  pt.get(kConfigKey_RcPitchDown, pitch_range_.lower, tobas_navio_ros::kPwmMax);
  pt.get(kConfigKey_RcPitchUp, pitch_range_.upper, tobas_navio_ros::kPwmMin);

  pt.get(kConfigKey_RcYawRight, yaw_range_.lower, tobas_navio_ros::kPwmMax);
  pt.get(kConfigKey_RcYawLeft, yaw_range_.upper, tobas_navio_ros::kPwmMin);

  pt.get(kConfigKey_RcThrustDown, thrust_range_.lower, tobas_navio_ros::kPwmMax);
  pt.get(kConfigKey_RcThrustUp, thrust_range_.upper, tobas_navio_ros::kPwmMin);

  pt.get(kConfigKey_RcModeProgram, modes_[tobas::kFlightModeProgram], tobas_navio_ros::kPwmMin);
  pt.get(kConfigKey_RcModeStabilize, modes_[tobas::kFlightModeStabilize], tobas_navio_ros::kPwmMid);
  pt.get(kConfigKey_RcModeAcrobat, modes_[tobas::kFlightModeAcrobat], tobas_navio_ros::kPwmMax);

  pt.get(kConfigKey_RcEStopOn, estop_on_, tobas_navio_ros::kPwmMin);
  pt.get(kConfigKey_RcEStopOff, estop_off_, tobas_navio_ros::kPwmMax);

  pt.get(kConfigKey_RcGPSwOn, gpsw_on_, tobas_navio_ros::kPwmMin);
  pt.get(kConfigKey_RcGPSwOff, gpsw_off_, tobas_navio_ros::kPwmMax);
}

bool RCInputHandler::reloadConfigCb(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
  reloadConfig();
}

void RCInputHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // Create message
  const auto rcin_msg = boost::make_shared<tobas_msgs::RCInput>();
  rcin_msg->header.stamp = event.current_real;

  // Roll
  if (rcin_.read(kRcChannelRoll) != navio::RCInput::E_NO_ERROR)
    rcin_msg->error.error = rcin_.getError();
  rcin_msg->roll = remap<double>(rcin_.getPeriod(), roll_range_.lower, roll_range_.upper, -1, 1);

  // Pitch
  if (rcin_.read(kRcChannelPitch) != navio::RCInput::E_NO_ERROR)
    rcin_msg->error.error = rcin_.getError();
  rcin_msg->pitch = remap<double>(rcin_.getPeriod(), pitch_range_.lower, pitch_range_.upper, -1, 1);

  // Yaw
  if (rcin_.read(kRcChannelYaw) != navio::RCInput::E_NO_ERROR)
    rcin_msg->error.error = rcin_.getError();
  rcin_msg->yaw = remap<double>(rcin_.getPeriod(), yaw_range_.lower, yaw_range_.upper, -1, 1);

  // Thrust
  if (rcin_.read(kRcChannelThrust) != navio::RCInput::E_NO_ERROR)
    rcin_msg->error.error = rcin_.getError();
  rcin_msg->thrust =
    remap<double>(rcin_.getPeriod(), thrust_range_.lower, thrust_range_.upper, 0, 1);

  // Mode
  if (rcin_.read(kRcChannelMode) != navio::RCInput::E_NO_ERROR)
    rcin_msg->error.error = rcin_.getError();
  rcin_msg->mode = tobas_std::closestIndex<double>(modes_, rcin_.getPeriod());

  // E-Stop
  if (rcin_.read(kRcChannelEStop) != navio::RCInput::E_NO_ERROR)
    rcin_msg->error.error = rcin_.getError();
  rcin_msg->e_stop = abs(rcin_.getPeriod() - estop_on_) < abs(rcin_.getPeriod() - estop_off_);

  // GPSw
  if (rcin_.read(kRcChannelGPSw) != navio::RCInput::E_NO_ERROR)
    rcin_msg->error.error = rcin_.getError();
  rcin_msg->gpsw = abs(rcin_.getPeriod() - gpsw_on_) < abs(rcin_.getPeriod() - gpsw_off_);

  // Error message
  if (rcin_msg->error.error != tobas_msgs::RCInputError::E_NO_ERROR)
    rosErrorThrottle(kErrorPeriod, name_, "Failed to read RC input.");

  // Publish message
  rcin_pub_.publish(rcin_msg);
}
}  // namespace tobas_navio_ros
