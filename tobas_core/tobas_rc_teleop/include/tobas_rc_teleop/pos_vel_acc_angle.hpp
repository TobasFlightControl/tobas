#pragma once

#include <tobas_command_msgs_adapter/angle.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PosVelAccAngleController : public BaseController
{
  using self = PosVelAccAngleController;
  using super = BaseController;

  static constexpr double kMaxPositionError = 5.;  // [m]

public:
  explicit PosVelAccAngleController();

  bool requirePosition() override;
  bool requireVelocity() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(tobas::BaseNode* node, tobas::FlightMode mode) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  rclcpp::Time t_last_rcin_;
  kdl::Vector tar_vel_G_;  // 地面座標系から見た目標速度
  kdl::Vector tar_pos_W_;  // 世界座標系から見た目標位置
  kdl::Euler tar_angle_;

  // rosparams
  double max_hor_vel_;    // [m/s]
  double max_ver_vel_;    // [m/s]
  double max_attitude_;   // [rad]
  double max_head_rate_;  // [rad/s]
  double hor_vel_expo_;
  double ver_vel_expo_;
  double atti_expo_;
  double head_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::PosVelAcc> pos_vel_acc_pub_;
  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;

  void publishPosVelAcc(
    const builtin_interfaces::msg::Time& stamp,
    const kdl::Vector& pos,
    const kdl::Vector& vel,
    const kdl::Vector& acc);
  void publishAngle(const builtin_interfaces::msg::Time& stamp, const kdl::Euler& angle);

  bool maxHorizontalVelocityCb(const double& p);
  bool maxVerticalVelocityCb(const double& p);
  bool maxAttitudeCb(const long& p);
  bool maxHeadingRateCb(const long& p);
  bool horizontalVelocityExpoCb(const long& p);
  bool verticalVelocityExpoCb(const long& p);
  bool attitudeExpoCb(const long& p);
  bool headingExpoCb(const long& p);
};
}  // namespace tobas_rc_teleop
