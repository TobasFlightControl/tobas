// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs/msg/roll_pitch_side_slip_throttle_vector.hpp>
#include <tobas_command_msgs_adapter/angle_throttle_vector.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class AngleSideSlipThrottleVectorController : public BaseController
{
  using self = AngleSideSlipThrottleVectorController;
  using super = BaseController;

  static constexpr double kMinThrustAngle =  0.0 / 180.0 * M_PI;
  static constexpr double kMaxThrustAngle = 90.0 / 180.0 * M_PI;
  static constexpr double kModeChangeThrustAngle = 30.0 / 180.0 * M_PI;

public:
  explicit AngleSideSlipThrottleVectorController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  enum ControlType {
    MULTICOPTER,
    FIXED_WING,
  };
  rclcpp::Time t_last_rcin_;
  traj::VelocityLimitedOnlineTrajectoryGenerator roll_filt_, pitch_filt_, thrust_angle_filt_;
  double tar_yaw_;
  ControlType last_control_type_ = ControlType::MULTICOPTER;

  // ROS parameters.
  double max_roll_;          // [rad]
  double max_pitch_;         // [rad]
  double max_side_slip_;     // [rad]
  double max_yaw_rate_;      // [rad/s]
  double roll_expo_;
  double pitch_expo_;
  double yaw_expo_;
  double throt_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::msg::RollPitchSideSlipThrottleVector> fixed_wing_cmd_pub_;
  ros2::PublisherPtr<tobas_command_msgs::AngleThrottleVector> multicopter_cmd_pub_;

  bool maxRollCb(const double& p);
  bool maxRollRateCb(const double& p);
  bool maxPitchCb(const double& p);
  bool maxPitchRateCb(const double& p);
  bool maxSideSlipCb(const double& p);
  bool maxYawRateCb(const double& p);
  bool maxThrustAngleRateCb(const double& p);
  bool rollExpoCb(const double& p);
  bool pitchExpoCb(const double& p);
  bool yawExpoCb(const double& p);
  bool throttleExpoCb(const double& p);
};
}  // namespace rc
}  // namespace tobas
