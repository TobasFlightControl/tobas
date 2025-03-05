#pragma once

#include <tobas_command_msgs_adapter/pose_twist_accel.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PoseTwistAccelController : public BaseController
{
  using super = BaseController;

public:
  explicit PoseTwistAccelController();

  bool requirePosition() override;
  bool requireOrientation() override;
  bool requireLinearVelocity() override;
  bool requireAngularVelocity() override;

  void initialize(tobas::BaseNode* node) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  // rosparams
  double max_hor_vel_;       // [m/s]
  double max_ver_vel_;       // [m/s]
  double max_attitude_;      // [rad]
  double max_heading_rate_;  // [rad/s]

  // Mutable
  bool is_up_commanded_;
  rclcpp::Time t_last_rcin_;
  kdl::Vector tar_vel_G_;  // 地面座標系から見た目標速度
  kdl::Vector tar_pos_W_;  // 世界座標系から見た目標位置
  kdl::Euler tar_rpy_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::PoseTwistAccel> cmd_pub_;

  void getStaticRosParams(tobas::BaseNode* node);
};
}  // namespace tobas_rc_teleop
