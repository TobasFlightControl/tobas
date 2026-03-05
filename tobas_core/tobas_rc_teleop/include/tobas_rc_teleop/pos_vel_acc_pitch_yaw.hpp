#pragma once

#include <tobas_command_msgs_adapter/pos_vel_acc_pitch_yaw.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PosVelAccPitchYawController : public BaseController
{
  using self = PosVelAccPitchYawController;
  using super = BaseController;

public:
  explicit PosVelAccPitchYawController();

  bool requirePosition() override;
  bool requireVelocity() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(tobas::BaseNode* node, tobas::FlightMode mode) override;
  void reset(const tobas_msgs::Odometry& odom, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  rclcpp::Time t_last_rcin_;
  kdl::Vector tar_vel_G_;  // 地面座標系から見た目標速度
  kdl::Vector tar_pos_W_;  // 世界座標系から見た目標位置
  double tar_pitch_;
  double tar_yaw_;

  // rosparams
  double max_hor_vel_;   // [m/s]
  double max_ver_vel_;   // [m/s]
  double max_pitch_;     // [rad]
  double max_yaw_rate_;  // [rad/s]
  double max_ep_down_;   // [m]
  double hor_vel_expo_;
  double ver_vel_expo_;
  double pitch_expo_;
  double yaw_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::PosVelAccPitchYaw> cmd_pub_;

  bool maxHorizontalVelocityCb(const double& p);
  bool maxVerticalVelocityCb(const double& p);
  bool maxPitchCb(const double& p);
  bool maxYawRateCb(const double& p);
  bool maxPositionErrorDown(const double& p);
  bool horizontalVelocityExpoCb(const double& p);
  bool verticalVelocityExpoCb(const double& p);
  bool pitchExpoCb(const double& p);
  bool yawExpoCb(const double& p);
};
}  // namespace tobas_rc_teleop
