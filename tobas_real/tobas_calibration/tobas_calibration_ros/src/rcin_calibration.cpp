#include "../include/tobas_calibration_ros/rcin_calibration.hpp"

using namespace std;

namespace tobas_calibration
{
RCInputCalibrationRos::RCInputCalibrationRos(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), pt_(tobas_navio_ros::kConfigPath)
{
  if (rcin_.initialize() < 0)
    TOBAS_EXIT("Failed to initialize RC input driver.");

  publish_timer_ = nh_.createTimer(kSamplingRate, &self::publishTimerCb, this, false, false);

  rcin_pub_ = nh_.advertise<tobas_calibration_msgs::RCInput>(kRcInputTopicName, 1);

  start_ss_ = nh_.advertiseService(kStartServiceName, &self::startServiceCb, this);
  finish_ss_ = nh_.advertiseService(kFinishServiceName, &self::finishServiceCb, this);
  cancel_ss_ = nh_.advertiseService(kCancelServiceName, &self::cancelServiceCb, this);
}

void RCInputCalibrationRos::publishTimerCb(const ros::TimerEvent& event)
{
  const auto rcin_msg = boost::make_shared<tobas_calibration_msgs::RCInput>();
  rcin_msg->header.stamp = event.current_real;

  for (size_t i = 0; i < navio::RCInput::channelCount(); ++i)
  {
    if (rcin_.read(i) < 0)
      rcin_msg->error.error = rcin_.getError();
    rcin_msg->data[i] = rcin_.getPeriod();
  }

  rcin_pub_.publish(rcin_msg);
}

bool RCInputCalibrationRos::startServiceCb(StartSrvType::Request&, StartSrvType::Response& res)
{
  publish_timer_.start();

  res.success = true;
  return true;
}

bool RCInputCalibrationRos::finishServiceCb(FinishSrvType::Request& req, FinishSrvType::Response& res)
{
  res.success = false;

  if (!publish_timer_.hasStarted())
  {
    res.message = "Data collecting timer is not running.";
    return true;
  }

  // 各チャンネルの値の範囲をチェック
  if (abs(req.roll_left - req.roll_right) < kMinSignalRange)
  {
    res.message = "The signals on Roll channel are too close.";
    return true;
  }
  if (abs(req.pitch_up - req.pitch_down) < kMinSignalRange)
  {
    res.message = "The signals on Pitch channel are too close.";
    return true;
  }
  if (abs(req.yaw_left - req.yaw_right) < kMinSignalRange)
  {
    res.message = "The signals on Yaw channel are too close.";
    return true;
  }
  if (abs(req.throttle_up - req.throttle_down) < kMinSignalRange)
  {
    res.message = "The signals on Throttle channel are too close.";
    return true;
  }
  if (abs(req.mode_program - req.mode_acrobat) < kMinSignalRange)
  {
    res.message = "The signals on Mode channel are too close.";
    return true;
  }
  if (abs(req.estop_on - req.estop_off) < kMinSignalRange)
  {
    res.message = "The signals on E-Stop channel are too close.";
    return true;
  }
  if (abs(req.gpsw_on - req.gpsw_off) < kMinSignalRange)
  {
    res.message = "The signals on GPSw channel are too close.";
    return true;
  }

  // Configに保存
  pt_.load();
  pt_.put(tobas_navio_ros::kConfigKey_RcRollLeft, req.roll_left);
  pt_.put(tobas_navio_ros::kConfigKey_RcRollRight, req.roll_right);
  pt_.put(tobas_navio_ros::kConfigKey_RcPitchUp, req.pitch_up);
  pt_.put(tobas_navio_ros::kConfigKey_RcPitchDown, req.pitch_down);
  pt_.put(tobas_navio_ros::kConfigKey_RcYawLeft, req.yaw_left);
  pt_.put(tobas_navio_ros::kConfigKey_RcYawRight, req.yaw_right);
  pt_.put(tobas_navio_ros::kConfigKey_RcThrottleUp, req.throttle_up);
  pt_.put(tobas_navio_ros::kConfigKey_RcThrottleDown, req.throttle_down);
  pt_.put(tobas_navio_ros::kConfigKey_RcModeProgram, req.mode_program);
  pt_.put(tobas_navio_ros::kConfigKey_RcModeStabilize, req.mode_stabilize);
  pt_.put(tobas_navio_ros::kConfigKey_RcModeAcrobat, req.mode_acrobat);
  pt_.put(tobas_navio_ros::kConfigKey_RcEStopOn, req.estop_on);
  pt_.put(tobas_navio_ros::kConfigKey_RcEStopOff, req.estop_off);
  pt_.put(tobas_navio_ros::kConfigKey_RcGPSwOn, req.gpsw_on);
  pt_.put(tobas_navio_ros::kConfigKey_RcGPSwOff, req.gpsw_off);
  pt_.save();

  publish_timer_.stop();
  res.success = true;
  return true;
}

bool RCInputCalibrationRos::cancelServiceCb(CancelSrvType::Request&, CancelSrvType::Response& res)
{
  publish_timer_.stop();

  res.success = true;
  return true;
}
}  // namespace tobas_calibration
