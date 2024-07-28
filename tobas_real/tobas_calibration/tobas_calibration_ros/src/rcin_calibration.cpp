#include <tobas_real_ros/common.hpp>

#include "../include/tobas_calibration_ros/rcin_calibration.hpp"

using namespace std;

namespace tobas_calibration
{
RCInputCalibrationRos::RCInputCalibrationRos(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), property_client_(nh_, tobas_real_ros::kPropertyServerFC)
{
  ss_ = nh_.advertiseService(kServiceName, &self::executeCb, this);
}

bool RCInputCalibrationRos::executeCb(SrvType::Request& req, SrvType::Response& res)
{
  // 各チャンネルの値の範囲をチェック
  if (abs(req.roll_left - req.roll_right) < kMinSignalRange)
  {
    res.success = false;
    res.message = "The signals on Roll channel are too close.";
    return true;
  }
  if (abs(req.pitch_up - req.pitch_down) < kMinSignalRange)
  {
    res.success = false;
    res.message = "The signals on Pitch channel are too close.";
    return true;
  }
  if (abs(req.yaw_left - req.yaw_right) < kMinSignalRange)
  {
    res.success = false;
    res.message = "The signals on Yaw channel are too close.";
    return true;
  }
  if (abs(req.throttle_up - req.throttle_down) < kMinSignalRange)
  {
    res.success = false;
    res.message = "The signals on Throttle channel are too close.";
    return true;
  }
  if (abs(req.mode_program - req.mode_acrobat) < kMinSignalRange)
  {
    res.success = false;
    res.message = "The signals on Mode channel are too close.";
    return true;
  }
  if (abs(req.estop_on - req.estop_off) < kMinSignalRange)
  {
    res.success = false;
    res.message = "The signals on E-Stop channel are too close.";
    return true;
  }
  if (abs(req.gpsw_on - req.gpsw_off) < kMinSignalRange)
  {
    res.success = false;
    res.message = "The signals on GPSw channel are too close.";
    return true;
  }

  // Configに保存
  if (property_client_.set(tobas_real_ros::kConfigKey_RcRollLeft, req.roll_left) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcRollRight, req.roll_right) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcPitchUp, req.pitch_up) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcPitchDown, req.pitch_down) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcYawLeft, req.yaw_left) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcYawRight, req.yaw_right) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcThrottleUp, req.throttle_up) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcThrottleDown, req.throttle_down) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcModeProgram, req.mode_program) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcModeStabilize, req.mode_stabilize) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcModeAcrobat, req.mode_acrobat) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcEStopOn, req.estop_on) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcEStopOff, req.estop_off) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcGPSwOn, req.gpsw_on) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.set(tobas_real_ros::kConfigKey_RcGPSwOff, req.gpsw_off) < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }
  if (property_client_.save() < 0)
  {
    res.success = false;
    res.message = property_client_.errorMessage();
    return true;
  }

  res.success = true;
  res.message = "";

  return true;
}
}  // namespace tobas_calibration
