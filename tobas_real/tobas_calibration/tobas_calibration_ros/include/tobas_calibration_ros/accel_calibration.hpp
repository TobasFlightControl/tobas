#pragma once

#include <array>

#include <tobas_algorithm/kahan.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_hal_msgs/Imu.h>
#include <tobas_calibration_msgs/AccelCalibration.h>

namespace tobas_calibration
{
class AccelCalibrationRos : public tobas::BaseNode
{
  static constexpr char kServiceName[] = "accel_calibration";

  static constexpr size_t kDataCount = 1000;
  static constexpr double kTimeout = 5.;  // [s]

  using super = tobas::BaseNode;
  using SrvType = tobas_calibration_msgs::AccelCalibration;

public:
  explicit AccelCalibrationRos(
    ros::NodeHandle& nh,
    ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  size_t cnt_;
  std::array<algo::Kahan<double>, 3> acc_sum_;
  Eigen::Vector3d acc_top_;

  ptree::PropertyClient property_client_;
  ros::ServiceServer ss_;

  bool getAccelMean(Eigen::Vector3d& des);

  void imuCb(const tobas_hal_msgs::ImuConstPtr& imu_raw);
  bool executeCb(SrvType::Request& req, SrvType::Response& res);
};
}  // namespace tobas_calibration
