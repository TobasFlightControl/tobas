// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs/msg/aile_elev_rud_throttle.hpp>
#include <tobas_command_msgs/msg/roll_pitch_side_slip_throttle_vector.hpp>
#include <tobas_command_msgs/msg/vel_roll_side_slip_mode.hpp>
#include <tobas_command_msgs_adapter/angle_throttle_vector.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc_pitch_yaw.hpp>

#include "./base_controller.hpp"
#include "./filter/second_order_velocity_filter.hpp"

namespace tobas::rc
{
class PosVelAccPitchYawRollSideSlipModeController : public BaseController
{
  static constexpr double kTiltStartAngle = 0.0;                 // [rad]
  static constexpr double kTiltFinishAngle = 0.5 * M_PI;         // [rad]
  static constexpr double kBackwardTransitionFinishSpeed = 2.0;  // [m/s]

  using self = PosVelAccPitchYawRollSideSlipModeController;
  using super = BaseController;

public:
  explicit PosVelAccPitchYawRollSideSlipModeController();

  bool requireHorizontalPosition() const override;
  bool requireVerticalPosition() const override;
  bool requireAttitude() const override;
  bool requireHeading() const override;

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  enum ControlMode
  {
    MULTICOPTER,
    FORWARD_FLIGHT,
    BACKWARD_TRANSITION,
  };

  ControlMode mode_ = ControlMode::MULTICOPTER;
  rclcpp::Time t_last_rcin_;
  SecondOrderVelocityFilter vx_filt_, vy_filt_, vz_filt_;
  traj::VelocityLimitedOnlineTrajectoryGenerator roll_filt_, pitch_filt_;
  kdl::Vector tar_pos_W_;
  double tar_yaw_;

  // ROS parameters.
  double max_hor_vel_;    // [m/s]
  double max_ver_vel_;    // [m/s]
  double max_roll_;       // [rad]
  double max_yaw_rate_;   // [rad/s]
  double max_side_slip_;  // [rad]
  double max_ep_down_;    // [m]
  double hor_vel_expo_;
  double ver_vel_expo_;
  double roll_expo_;
  double yaw_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::PosVelAccPitchYaw> multicopter_cmd_pub_;
  ros2::PublisherPtr<tobas_command_msgs::msg::VelRollSideSlipMode> fixed_wing_cmd_pub_;

  // Subscriber
  ros2::SubscriberPtr<tobas_command_msgs::AngleThrottleVector> multicopter_angle_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::AileElevRudThrottle> fixed_wing_manual_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::RollPitchSideSlipThrottleVector> fixed_wing_angle_cmd_sub_;

  void multicopterAngleCmdCb(const tobas_command_msgs::AngleThrottleVector::ConstSharedPtr& _cmd);
  void fixedWingManualCmdCb(const tobas_command_msgs::msg::AileElevRudThrottle::ConstSharedPtr& _cmd);
  void fixedWingAngleCmdCb(const tobas_command_msgs::msg::RollPitchSideSlipThrottleVector::ConstSharedPtr& _cmd);

  void resetFixedWingControllers(const tobas_msgs::Odometry& setpoint, bool landed);
  void resetMulticopterControllers(const tobas_msgs::Odometry& setpoint, bool landed);
  void maxHorizontalVelocityCb(double _p);
  void maxHorizontalJerkCb(double _p);
  void maxVerticalVelocityCb(double _p);
  void maxVerticalJerkCb(double _p);
  void maxRollCb(double _p);
  void maxRollRateCb(double _p);
  void maxPitchRateCb(double _p);
  void maxYawRateCb(double _p);
  void maxSideSlipCb(double _p);
  void maxPositionErrorDown(double _p);
  void horizontalVelocityExpoCb(double _p);
  void verticalVelocityExpoCb(double _p);
  void rollExpoCb(double _p);
  void yawExpoCb(double _p);
};
}  // namespace tobas::rc
