#pragma once

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <navio2/RCInput.h>
#include <tobas_real/common.hpp>

#include <tobas_calibration_msgs/RCInputCalibration.h>

namespace tobas_calibration
{
class RCInputCalibrationRos
{
  static constexpr char kRcInputTopicName[] = "rcin_calibration/rc_input_raw";
  static constexpr char kStartServiceName[] = "rcin_calibration/start";
  static constexpr char kFinishServiceName[] = "rcin_calibration/finish";
  static constexpr char kCancelServiceName[] = "rcin_calibration/cancel";

  static constexpr size_t kSamplingRate = 100;  // [Hz]
  static constexpr size_t kMinSignalRange = 300;

  using self = RCInputCalibrationRos;
  using StartSrvType = std_srvs::Trigger;
  using FinishSrvType = tobas_calibration_msgs::RCInputCalibration;
  using CancelSrvType = std_srvs::Trigger;

public:
  explicit RCInputCalibrationRos(ros::NodeHandle& nh);

private:
  RCInput rcin_;

  ros::Timer publish_timer_;
  ros::Publisher rcin_pub_;
  ros::ServiceServer start_ss_;
  ros::ServiceServer finish_ss_;
  ros::ServiceServer cancel_ss_;

  void publishTimerCb(const ros::TimerEvent& event);

  bool startServiceCb(StartSrvType::Request& req, StartSrvType::Response& res);
  bool finishServiceCb(FinishSrvType::Request& req, FinishSrvType::Response& res);
  bool cancelServiceCb(CancelSrvType::Request& req, CancelSrvType::Response& res);
};
}  // namespace tobas_calibration
