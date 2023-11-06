#pragma once

#include <dh_std_tools/range.hpp>

#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/RCInput.h>
#include <tobas_msgs/PositionYaw.h>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PositionYawController : public BaseController
{
  using super = BaseController;

public:
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;
  void reset(const tobas_msgs::PoseTwist& pt) override;
  void update(
    const tobas_msgs::RCInput& rcin,
    const tobas_msgs::PoseTwist& pt,
    const double& battery_voltage,
    const dh_std::Range<double>& dead_zone) override;

private:
  tobas_msgs::PositionYaw pos_yaw_;
  KDL::Vector vel_;
  ros::Time t_last_rcin_;
  KDL::Vector max_pos_err_;

  // rosparams
  double max_hor_vel_;      // [m/s]
  double max_ver_vel_;      // [m/s]
  double max_yawrate_;      // [rad/s]
  double max_hor_pos_err_;  // [m]
  double max_ver_pos_err_;  // [m]
  double max_yaw_err_;      // [rad]

  // Publisher
  ros::Publisher pos_yaw_pub_;

  void getRosParams(ros::NodeHandle& pnh);
  void registerPublishers(ros::NodeHandle& nh);
};
}  // namespace tobas_rc_teleop
