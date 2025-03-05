#pragma once

#include <tobas_command_msgs_adapter/accel_yaw.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class AccelYawController : public BaseController
{
  using super = BaseController;

public:
  explicit AccelYawController();

  bool requirePosition() override;
  bool requireOrientation() override;
  bool requireLinearVelocity() override;
  bool requireAngularVelocity() override;

  void initialize(tobas::BaseNode* node) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  bool is_up_commanded_;
  builtin_interfaces::msg::Time t_last_rcin_;
  kdl::Vector tar_acc_G_;
  double tar_yaw_;

  // rosparams
  double max_hor_acc_;       // [m/s]
  double max_ver_acc_;       // [m/s]
  double max_heading_rate_;  // [rad/s]

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::AccelYaw> cmd_pub_;

  void getStaticRosParams(tobas::BaseNode* node);
};
}  // namespace tobas_rc_teleop
