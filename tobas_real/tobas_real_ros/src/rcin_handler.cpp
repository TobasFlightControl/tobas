#include <tobas_math/core.hpp>
#include <tobas_std_tools/array.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_msgs/RCInput.h>

#include "../include/tobas_real_ros/rcin_handler.hpp"
#include "../include/tobas_real_ros/common.hpp"

using namespace std;

namespace tobas_real_ros
{
RCInputHandler::RCInputHandler(, const string& name)
  : super(node, pnh, name), property_client_(node_, kPropertyServerFC)
{
  reloadConfig();

  rcin_pub_ = node_.advertise<tobas_msgs::RCInput>(tobas::kRcInputTopic, 1);
  sbus_sub_ = node_.subscribe(hal::kSbusTopic, 1, &self::sbusCb, this, tcpNoDelay());

  reload_config_srv_ = node_.advertiseService(name + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);
}

void RCInputHandler::setToDefaults()
{
  roll_range_.set(tobas::kPwmMin, tobas::kPwmMax);
  pitch_range_.set(tobas::kPwmMax, tobas::kPwmMin);
  yaw_range_.set(tobas::kPwmMax, tobas::kPwmMin);
  throttle_range_.set(tobas::kPwmMax, tobas::kPwmMin);

  modes_[tobas::kFlightModeProgram] = tobas::kPwmMin;
  modes_[tobas::kFlightModeStabilize] = tobas::kPwmMid;
  modes_[tobas::kFlightModeAcrobat] = tobas::kPwmMax;

  estop_on_ = tobas::kPwmMin;
  estop_off_ = tobas::kPwmMax;
  gpsw_on_ = tobas::kPwmMin;
  gpsw_off_ = tobas::kPwmMax;
}

bool RCInputHandler::reloadConfig()
{
  if (property_client_.get(kConfigKey_RcRollLeft, roll_range_.lower) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_.get(kConfigKey_RcRollRight, roll_range_.upper) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }

  if (property_client_.get(kConfigKey_RcPitchDown, pitch_range_.lower) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_.get(kConfigKey_RcPitchUp, pitch_range_.upper) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }

  if (property_client_.get(kConfigKey_RcYawRight, yaw_range_.lower) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_.get(kConfigKey_RcYawLeft, yaw_range_.upper) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }

  if (property_client_.get(kConfigKey_RcThrottleDown, throttle_range_.lower) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_.get(kConfigKey_RcThrottleUp, throttle_range_.upper) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }

  if (property_client_.get(kConfigKey_RcModeProgram, modes_[tobas::kFlightModeProgram]) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_.get(kConfigKey_RcModeStabilize, modes_[tobas::kFlightModeStabilize]) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_.get(kConfigKey_RcModeAcrobat, modes_[tobas::kFlightModeAcrobat]) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }

  if (property_client_.get(kConfigKey_RcEStopOn, estop_on_) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_.get(kConfigKey_RcEStopOff, estop_off_) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }

  if (property_client_.get(kConfigKey_RcGPSwOn, gpsw_on_) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_.get(kConfigKey_RcGPSwOff, gpsw_off_) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    setToDefaults();
    return false;
  }

  return true;
}

void RCInputHandler::sbusCb(const tobas_hal_msgs::SbusConstPtr& sbus)
{
  // Create message
  const auto rcin_msg = make_unique<tobas_msgs::RCInput>();

  // Fill header
  rcin_msg->header = sbus->header;

  // Fill duty periods for each channel
  rcin_msg->roll = math::remap<double>(
    sbus->data[kRcChannelRoll], roll_range_.lower, roll_range_.upper, tobas::kRCInputMin, tobas::kRCInputMax);
  rcin_msg->pitch = math::remap<double>(
    sbus->data[kRcChannelPitch], pitch_range_.lower, pitch_range_.upper, tobas::kRCInputMin, tobas::kRCInputMax);
  rcin_msg->yaw = math::remap<double>(
    sbus->data[kRcChannelYaw], yaw_range_.lower, yaw_range_.upper, tobas::kRCInputMin, tobas::kRCInputMax);
  rcin_msg->throttle = math::remap<double>(
    sbus->data[kRcChannelThrottle], throttle_range_.lower, throttle_range_.upper, tobas::kRCInputMin,
    tobas::kRCInputMax);
  rcin_msg->mode = tobas_std::closestIndex(modes_, sbus->data[kRcChannelMode]);
  rcin_msg->e_stop = abs(sbus->data[kRcChannelEStop] - estop_on_) < abs(sbus->data[kRcChannelEStop] - estop_off_);
  rcin_msg->gpsw = abs(sbus->data[kRcChannelGPSw] - gpsw_on_) < abs(sbus->data[kRcChannelGPSw] - gpsw_off_);

  // Publish message
  rcin_pub_.publish(rcin_msg);
}

bool RCInputHandler::reloadConfigCb(std_srvs::srv::Trigger::Request&, std_srvs::srv::Trigger::Response& res)
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
}  // namespace tobas_real_ros
