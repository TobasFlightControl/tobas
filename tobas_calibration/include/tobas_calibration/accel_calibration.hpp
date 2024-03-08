#pragma once

#include <Eigen/Core>
#include <ros/ros.h>

#include <tobas_real/common.hpp>

#include <tobas_calibration/AccelCalibration.h>

namespace tobas_calibration
{
class AccelCalibrationRos
{
  static constexpr char kServiceName[] = "accel_calibration";

  static constexpr size_t kDataCount = 1000;
  static constexpr size_t kSleepTime = 10000;  // [us]

public:
  explicit AccelCalibrationRos(ros::NodeHandle& nh);

private:
  tobas_real::ImuDevice imu_;
  Eigen::Vector3f acc_;

  ros::ServiceServer ss_;

  Eigen::Vector3f readAccel();
  bool executeCb(
    tobas_calibration::AccelCalibrationRequest& req,
    tobas_calibration::AccelCalibrationResponse& res);
};
}  // namespace tobas_calibration
