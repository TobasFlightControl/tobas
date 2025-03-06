#pragma once

#include <tobas_command_msgs_adapter/pos_vel.hpp>
#include <tobas_command_msgs_adapter/angle.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PosVelAngleController : public BaseController
{
  using super = BaseController;

public:
  explicit PosVelAngleController();

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
  kdl::Euler tar_angle_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::PosVel> pos_vel_pub_;
  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;

  void getStaticRosParams(tobas::BaseNode* node);

  void publishPosVel(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& pos, const kdl::Vector& vel);
  void publishAngle(const builtin_interfaces::msg::Time& stamp, const kdl::Euler& angle);
};
}  // namespace tobas_rc_teleop
