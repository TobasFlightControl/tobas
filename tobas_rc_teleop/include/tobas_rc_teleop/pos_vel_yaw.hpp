#pragma once

#include <tobas_command_msgs_adapter/pos_vel_yaw.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PosVelYawController : public BaseController
{
  using super = BaseController;

public:
  explicit PosVelYawController();

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
  kdl::Vector tar_vel_G_;  // 地面座標系から見た目標速度
  kdl::Vector tar_pos_W_;  // 世界座標系から見た目標位置
  double tar_yaw_;

  // rosparams
  double max_hor_vel_;       // [m/s]
  double max_ver_vel_;       // [m/s]
  double max_heading_rate_;  // [rad/s]

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::PosVelYaw> cmd_pub_;

  void getStaticRosParams(tobas::BaseNode* node);
};
}  // namespace tobas_rc_teleop
