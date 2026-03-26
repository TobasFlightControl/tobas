#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs_adapter/accel.hpp>
#include <tobas_command_msgs_adapter/angle.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class AccelAngleController : public BaseController
{
  using self = AccelAngleController;
  using super = BaseController;

public:
  explicit AccelAngleController();

  bool requirePosition() override;
  bool requireVelocity() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(tobas::BaseNode* node, tobas::FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  rclcpp::Time t_last_rcin_;
  traj::VelocityLimitedOnlineTrajectoryGenerator ax_filt_, ay_filt_, roll_filt_, pitch_filt_;
  double tar_yaw_;

  // rosparams
  double max_hor_acc_;    // [m/s]
  double max_ver_acc_;    // [m/s]
  double max_attitude_;   // [rad]
  double max_head_rate_;  // [rad/s]
  double hor_acc_expo_;
  double ver_acc_expo_;
  double atti_expo_;
  double head_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::Accel> accel_pub_;
  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;

  void publishAccel(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& acc);
  void publishAngle(const builtin_interfaces::msg::Time& stamp, double roll, double pitch, double yaw);

  bool maxHorizontalAccelCb(const double& p);
  bool maxHorizontalJerkCb(const double& p);
  bool maxVerticalAccelCb(const double& p);
  bool maxAttitudeCb(const double& p);
  bool maxAttitudeRateCb(const double& p);
  bool maxHeadingRateCb(const double& p);
  bool horizontalAccelExpoCb(const double& p);
  bool verticalAccelExpoCb(const double& p);
  bool attitudeExpoCb(const double& p);
  bool headingExpoCb(const double& p);
};
}  // namespace rc
}  // namespace tobas
