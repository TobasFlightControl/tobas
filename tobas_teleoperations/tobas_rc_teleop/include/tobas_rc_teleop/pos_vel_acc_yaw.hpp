#pragma once

#include <tobas_command_msgs_adapter/pos_vel_acc_yaw.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PosVelAccYawController : public BaseController
{
  using super = BaseController;

public:
  explicit PosVelAccYawController();

  void initialize(tobas::BaseNode* node) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  bool is_up_commanded_;
  builtin_interfaces::msg::Time t_last_rcin_;
  kdl::Vector tar_vel_F_;
  kdl::Vector tar_pos_W_;
  double tar_yaw_;

  // rosparams
  double max_hor_vel_;  // [m/s]
  double max_ver_vel_;  // [m/s]
  double max_yawrate_;  // [rad/s]

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::PosVelAccYaw> cmd_pub_;

  void getStaticRosParams(tobas::BaseNode* node);
};
}  // namespace tobas_rc_teleop
