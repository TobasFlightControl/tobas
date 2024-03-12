#pragma once

#include <Eigen/Core>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <tobas_real/common.hpp>
#include <tobas_real/ellipse_transformer.hpp>

#include <tobas_calibration_msgs/MagCalibration.h>

namespace tobas_calibration
{
class MagCalibrationRos
{
  static constexpr char kMagTopicName[] = "mag_calibration/magnetic_field_raw";
  static constexpr char kStartServiceName[] = "mag_calibration/start";
  static constexpr char kFinishServiceName[] = "mag_calibration/finish";
  static constexpr char kCancelServiceName[] = "mag_calibration/cancel";

  static constexpr size_t kSamplingRate = 100;     // [Hz]
  static constexpr size_t kMaxDataCount = 100000;  // 4 * 3 * 100000 / 1000000 = 1.2MB

  using self = MagCalibrationRos;
  using StartSrvType = std_srvs::Trigger;
  using FinishSrvType = tobas_calibration_msgs::MagCalibration;
  using CancelSrvType = std_srvs::Trigger;

public:
  explicit MagCalibrationRos(ros::NodeHandle& nh);

private:
  tobas_real::ImuDevice imu_;
  float mx_, my_, mz_;
  std::vector<Eigen::Vector3f> mag_data_;
  tobas_real::EllipseTransformer mag_trans_;

  ros::Timer collect_data_timer_;
  ros::Publisher mag_pub_;
  ros::ServiceServer start_ss_;
  ros::ServiceServer finish_ss_;
  ros::ServiceServer cancel_ss_;

  void collectDataTimerCb(const ros::TimerEvent& event);

  bool startServiceCb(StartSrvType::Request& req, StartSrvType::Response& res);
  bool finishServiceCb(FinishSrvType::Request& req, FinishSrvType::Response& res);
  bool cancelServiceCb(CancelSrvType::Request& req, CancelSrvType::Response& res);
};
}  // namespace tobas_calibration
