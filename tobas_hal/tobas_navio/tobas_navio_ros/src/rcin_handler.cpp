#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/array.hpp>
#include <tobas_std_tools/property_tree.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_msgs/RCInput.h>

#include "../include/tobas_navio_ros/rcin_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;
using namespace tobas_std;

namespace tobas_navio_ros
{
RCInputHandler::RCInputHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  PRINT_DEBUG("RCInputHandler::RCInputHandler");

  reloadConfig();

  if (rcin_.initialize() != navio::RCInput::E_NO_ERROR)
    TOBAS_EXIT("Failed to initialize RC input driver.");

  rcin_pub_ = nh_.advertise<tobas_msgs::RCInput>(tobas::kRcInputTopic, 1);
  reload_config_srv_ = nh_.advertiseService(name + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);

  PRINT_DEBUG("/RCInputHandler::RCInputHandler");
}

bool RCInputHandler::reloadConfig()
{
  PRINT_DEBUG("RCInputHandler::reloadConfig");

  PropertyTree pt(kConfigPath);

  if (!pt.get(kConfigKey_RcRollLeft, roll_range_.lower, tobas_navio_ros::kPwmMin))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcRollLeft, ".");
    return false;
  }
  if (!pt.get(kConfigKey_RcRollRight, roll_range_.upper, tobas_navio_ros::kPwmMax))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcRollRight, ".");
    return false;
  }

  if (!pt.get(kConfigKey_RcPitchDown, pitch_range_.lower, tobas_navio_ros::kPwmMax))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcPitchDown, ".");
    return false;
  }
  if (!pt.get(kConfigKey_RcPitchUp, pitch_range_.upper, tobas_navio_ros::kPwmMin))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcPitchUp, ".");
    return false;
  }

  if (!pt.get(kConfigKey_RcYawRight, yaw_range_.lower, tobas_navio_ros::kPwmMax))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcYawRight, ".");
    return false;
  }
  if (!pt.get(kConfigKey_RcYawLeft, yaw_range_.upper, tobas_navio_ros::kPwmMin))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcYawLeft, ".");
    return false;
  }

  if (!pt.get(kConfigKey_RcThrottleDown, throttle_range_.lower, tobas_navio_ros::kPwmMax))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcThrottleDown, ".");
    return false;
  }
  if (!pt.get(kConfigKey_RcThrottleUp, throttle_range_.upper, tobas_navio_ros::kPwmMin))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcThrottleUp, ".");
    return false;
  }

  if (!pt.get(kConfigKey_RcModeProgram, modes_[tobas::kFlightModeProgram], tobas_navio_ros::kPwmMin))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcModeProgram, ".");
    return false;
  }
  if (!pt.get(kConfigKey_RcModeStabilize, modes_[tobas::kFlightModeStabilize], tobas_navio_ros::kPwmMid))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcModeStabilize, ".");
    return false;
  }
  if (!pt.get(kConfigKey_RcModeAcrobat, modes_[tobas::kFlightModeAcrobat], tobas_navio_ros::kPwmMax))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcModeAcrobat, ".");
    return false;
  }

  if (!pt.get(kConfigKey_RcEStopOn, estop_on_, tobas_navio_ros::kPwmMin))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcEStopOn, ".");
    return false;
  }
  if (!pt.get(kConfigKey_RcEStopOff, estop_off_, tobas_navio_ros::kPwmMax))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcEStopOff, ".");
    return false;
  }

  if (!pt.get(kConfigKey_RcGPSwOn, gpsw_on_, tobas_navio_ros::kPwmMin))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcGPSwOn, ".");
    return false;
  }
  if (!pt.get(kConfigKey_RcGPSwOff, gpsw_off_, tobas_navio_ros::kPwmMax))
  {
    TOBAS_ERROR("Failed to get ", kConfigKey_RcGPSwOff, ".");
    return false;
  }

  return true;
}

bool RCInputHandler::reloadConfigCb(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res)
{
  if (!reloadConfig())
  {
    res.success = false;
    res.message = "Failed to reload configurations.";
    return true;
  }

  res.success = true;
  return true;
}

void RCInputHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // Create message
  const auto rcin_msg = boost::make_shared<tobas_msgs::RCInput>();
  rcin_msg->header.stamp = event.current_real;

  // Roll
  if (rcin_.read(kRcChannelRoll) != navio::RCInput::E_NO_ERROR)
    rcin_msg->error.error = rcin_.getError();
  rcin_msg->roll =
    remap<double>(rcin_.getPeriod(), roll_range_.lower, roll_range_.upper, tobas::kRCInputMin, tobas::kRCInputMax);

  // Pitch
  if (rcin_.read(kRcChannelPitch) != navio::RCInput::E_NO_ERROR)
    rcin_msg->error.error = rcin_.getError();
  rcin_msg->pitch =
    remap<double>(rcin_.getPeriod(), pitch_range_.lower, pitch_range_.upper, tobas::kRCInputMin, tobas::kRCInputMax);

  // Yaw
  if (rcin_.read(kRcChannelYaw) != navio::RCInput::E_NO_ERROR)
    rcin_msg->error.error = rcin_.getError();
  rcin_msg->yaw =
    remap<double>(rcin_.getPeriod(), yaw_range_.lower, yaw_range_.upper, tobas::kRCInputMin, tobas::kRCInputMax);

  // Throttle
  if (rcin_.read(kRcChannelThrottle) != navio::RCInput::E_NO_ERROR)
    rcin_msg->error.error = rcin_.getError();
  rcin_msg->throttle = remap<double>(
    rcin_.getPeriod(), throttle_range_.lower, throttle_range_.upper, tobas::kRCInputMin, tobas::kRCInputMax);

  // Mode
  if (rcin_.read(kRcChannelMode) != navio::RCInput::E_NO_ERROR)
    rcin_msg->error.error = rcin_.getError();
  rcin_msg->mode = closestIndex<double>(modes_, rcin_.getPeriod());

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
    TOBAS_ERROR_THROTTLE(kErrorPeriod, "Failed to read RC input.");

  // Publish message
  rcin_pub_.publish(rcin_msg);
}
}  // namespace tobas_navio_ros
