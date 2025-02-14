#pragma once

#include <tobas_command_msgs/msg/angle_throttle.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class AngleThrottleController : public BaseController
{
  using super = BaseController;

public:
  explicit AngleThrottleController();

  bool requirePosition() override;
  bool requireOrientation() override;
  bool requireLinearVelocity() override;
  bool requireAngularVelocity() override;

  void initialize(tobas::BaseNode* node) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  double yaw_;
  rclcpp::Time t_last_rcin_;

  // rosparams
  double max_attitude_;      // [rad]
  double max_heading_rate_;  // [rad/s]

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::msg::AngleThrottle> cmd_pub_;

  void getStaticRosParams(tobas::BaseNode* node);
};
}  // namespace tobas_rc_teleop
