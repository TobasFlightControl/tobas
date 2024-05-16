#pragma once

#include <Eigen/Core>
#include <ros/ros.h>

#include <tobas_navio_ros/common.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_calibration_msgs/AccelCalibration.h>

namespace tobas_calibration
{
class AccelCalibrationRos : public tobas::BaseNode
{
  static constexpr char kServiceName[] = "accel_calibration";

  static constexpr size_t kDataCount = 1000;
  static constexpr size_t kSleepTime = 10;  // [ms]

  using super = tobas::BaseNode;
  using SrvType = tobas_calibration_msgs::AccelCalibration;

public:
  explicit AccelCalibrationRos(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas_navio_ros::ImuDevice imu_;
  Eigen::Vector3f acc_;

  ros::ServiceServer ss_;

  Eigen::Vector3f readAccel();
  bool executeCb(SrvType::Request& req, SrvType::Response& res);
};
}  // namespace tobas_calibration
