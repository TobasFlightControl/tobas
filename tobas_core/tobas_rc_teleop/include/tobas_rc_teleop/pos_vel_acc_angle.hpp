#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs_adapter/angle.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
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

  void initialize(BaseNode* node, tobas::FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  rclcpp::Time t_last_rcin_;
  traj::VelocityLimitedOnlineTrajectoryGenerator vx_filt_, vy_filt_, vz_filt_, roll_filt_, pitch_filt_;
  kdl::Vector tar_pos_W_;
  double tar_yaw_;

  // rosparams
  double max_hor_vel_;    // [m/s]
  double max_ver_vel_;    // [m/s]
  double max_attitude_;   // [rad]
  double max_head_rate_;  // [rad/s]
  double max_ep_down_;    // [m]
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
  void publishAngle(const builtin_interfaces::msg::Time& stamp, double roll, double pitch, double yaw);

  bool maxHorizontalVelocityCb(const double& p);
  bool maxHorizontalAccelCb(const double& p);
  bool maxVerticalVelocityCb(const double& p);
  bool maxVerticalAccelCb(const double& p);
  bool maxAttitudeCb(const double& p);
  bool maxAttitudeRateCb(const double& p);
  bool maxHeadingRateCb(const double& p);
  bool maxPositionErrorDown(const double& p);
  bool horizontalVelocityExpoCb(const double& p);
  bool verticalVelocityExpoCb(const double& p);
  bool attitudeExpoCb(const double& p);
  bool headingExpoCb(const double& p);
};
}  // namespace rc
}  // namespace tobas
