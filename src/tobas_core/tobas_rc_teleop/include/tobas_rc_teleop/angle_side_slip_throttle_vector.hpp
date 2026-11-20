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

  static constexpr double kMinThrustAngle = 0.0 / 180.0 * M_PI;
  static constexpr double kMaxThrustAngle = 90.0 / 180.0 * M_PI;
  static constexpr double kModeChangeThrustAngle = 30.0 / 180.0 * M_PI;

public:
  explicit AngleSideSlipThrottleVectorController();

  bool requireHorizontalPosition() const override;
  bool requireVerticalPosition() const override;
  bool requireAttitude() const override;
  bool requireHeading() const override;

  void initialize(BaseNode* _node, FlightMode _mode) override;
  void reset(const builtin_interfaces::msg::Time& _stamp, const tobas_msgs::Odometry& _setpoint, bool _landed) override;
  void update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry& _odom, bool _landed) override;

private:
  enum ControlType
  {
    MULTICOPTER,
    FIXED_WING,
  };
  rclcpp::Time t_last_rcin_;
  traj::VelocityLimitedOnlineTrajectoryGenerator roll_filt_, pitch_filt_, thrust_angle_filt_;
  double tar_yaw_;
  ControlType last_control_type_ = ControlType::MULTICOPTER;

  // ROS parameters.
  double max_roll_;       // [rad]
  double max_pitch_;      // [rad]
  double max_side_slip_;  // [rad]
  double max_yaw_rate_;   // [rad/s]
  double roll_expo_;
  double pitch_expo_;
  double yaw_expo_;
  double throt_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::msg::RollPitchSideSlipThrottleVector> fixed_wing_cmd_pub_;
  ros2::PublisherPtr<tobas_command_msgs::AngleThrottleVector> multicopter_cmd_pub_;

  void maxRollCb(double _p);
  void maxRollRateCb(double _p);
  void maxPitchCb(double _p);
  void maxPitchRateCb(double _p);
  void maxSideSlipCb(double _p);
  void maxYawRateCb(double _p);
  void maxThrustAngleRateCb(double _p);
  void rollExpoCb(double _p);
  void pitchExpoCb(double _p);
  void yawExpoCb(double _p);
  void throttleExpoCb(double _p);
};
}  // namespace rc
}  // namespace tobas
