#pragma once

#include <Eigen/Core>
#include <ros/ros.h>

#include <tobas_real/common.hpp>
#include <tobas_real/ellipse_transformer.hpp>

#include <tobas_calibration/StartMagCalibration.h>
#include <tobas_calibration/FinishMagCalibration.h>
#include <tobas_calibration/CancelMagCalibration.h>

namespace tobas_calibration
{
class MagCalibrationRos
{
  static constexpr char kStartServiceName[] = "start_mag_calibration";
  static constexpr char kFinishServiceName[] = "finish_mag_calibration";
  static constexpr char kCancelServiceName[] = "cancel_mag_calibration";

  static constexpr size_t kSamplingRate = 100;     // [Hz]
  static constexpr size_t kMaxDataCount = 100000;  // 4 * 3 * 100000 / 1000000 = 1.2MB

  using self = MagCalibrationRos;
  using StartSrvType = tobas_calibration::StartMagCalibration;
  using FinishSrvType = tobas_calibration::FinishMagCalibration;
  using CancelSrvType = tobas_calibration::CancelMagCalibration;

public:
  explicit MagCalibrationRos(ros::NodeHandle& nh);

private:
  tobas_real::ImuDevice imu_;
  float mx_, my_, mz_;
  std::vector<Eigen::Vector3f> mag_data_;
  tobas_real::EllipseTransformer mag_trans_;

  ros::Timer collect_data_timer_;

  ros::ServiceServer start_ss_;
  ros::ServiceServer finish_ss_;
  ros::ServiceServer cancel_ss_;

  void collectDataTimerCb(const ros::TimerEvent& event);

  bool startServiceCb(StartSrvType::Request& req, StartSrvType::Response& res);
  bool finishServiceCb(FinishSrvType::Request& req, FinishSrvType::Response& res);
  bool cancelServiceCb(CancelSrvType::Request& req, CancelSrvType::Response& res);
};
}  // namespace tobas_calibration
