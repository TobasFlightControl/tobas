#pragma once

#include <tobas_command_msgs_adapter/accel_yaw.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class AccelYawController : public BaseController
{
  using self = AccelYawController;
  using super = BaseController;

public:
  explicit AccelYawController();

  bool requirePosition() override;
  bool requireOrientation() override;
  bool requireLinearVelocity() override;
  bool requireAngularVelocity() override;

  void initialize(tobas::BaseNode* node, tobas::flight_mode_t mode) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  builtin_interfaces::msg::Time t_last_rcin_;
  kdl::Vector tar_acc_G_;
  double tar_yaw_;

  // rosparams
  double max_hor_acc_;    // [m/s]
  double max_ver_acc_;    // [m/s]
  double max_head_rate_;  // [rad/s]
  double hor_acc_expo_;
  double ver_acc_expo_;
  double head_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::AccelYaw> cmd_pub_;

  bool maxHorizontalAccelCb(const double& p);
  bool maxVerticalAccelCb(const double& p);
  bool maxHeadingRateCb(const double& p);
  bool horizontalAccelExpoCb(const long& p);
  bool verticalAccelExpoCb(const long& p);
  bool headingExpoCb(const long& p);
};
}  // namespace tobas_rc_teleop
