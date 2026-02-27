#pragma once

#include <tobas_command_msgs_adapter/pos_vel_acc_yaw.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PosVelAccYawController : public BaseController
{
  using self = PosVelAccYawController;
  using super = BaseController;

  static constexpr double kMaxPositionError = 5.;  // [m]

public:
  explicit PosVelAccYawController();

  bool requirePosition() override;
  bool requireVelocity() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(tobas::BaseNode* node, tobas::FlightMode mode) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  builtin_interfaces::msg::Time t_last_rcin_;
  kdl::Vector tar_vel_G_;  // 地面座標系から見た目標速度
  kdl::Vector tar_pos_W_;  // 世界座標系から見た目標位置
  double tar_yaw_;

  // rosparams
  double max_hor_vel_;    // [m/s]
  double max_ver_vel_;    // [m/s]
  double max_head_rate_;  // [rad/s]
  double hor_vel_expo_;
  double ver_vel_expo_;
  double head_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::PosVelAccYaw> cmd_pub_;

  bool maxHorizontalVelocityCb(const double& p);
  bool maxVerticalVelocityCb(const double& p);
  bool maxHeadingRateCb(const long& p);
  bool horizontalVelocityExpoCb(const long& p);
  bool verticalVelocityExpoCb(const long& p);
  bool headingExpoCb(const long& p);
};
}  // namespace tobas_rc_teleop
