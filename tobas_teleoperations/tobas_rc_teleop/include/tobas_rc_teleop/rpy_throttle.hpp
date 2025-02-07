#pragma once

#include <tobas_command_msgs_adapter/roll_pitch_yaw_throttle.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class RollPitchYawThrottleController : public BaseController
{
  using super = BaseController;

public:
  explicit RollPitchYawThrottleController();

  void initialize(tobas::BaseNode* node) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  double yaw_;
  rclcpp::Time t_last_rcin_;

  // rosparams
  double max_attitude_;  // [rad]
  double max_yawrate_;   // [rad/s]

  // PubSub
  ros2::PublisherPtr<tobas_msgs::RollPitchYawThrottle> rpyt_pub_;

  void getStaticRosParams(tobas::BaseNode* node);
};
}  // namespace tobas_rc_teleop
