#pragma once

#include <tobas_command_msgs_adapter/accel.hpp>
#include <tobas_command_msgs_adapter/angle.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class AccelAngleController : public BaseController
{
  using super = BaseController;

public:
  explicit AccelAngleController();

  bool requirePosition() override;
  bool requireOrientation() override;
  bool requireLinearVelocity() override;
  bool requireAngularVelocity() override;

  void initialize(tobas::BaseNode* node) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  rclcpp::Time t_last_rcin_;
  kdl::Vector tar_acc_G_;
  kdl::Euler tar_angle_;

  // rosparams
  double max_hor_acc_;    // [m/s]
  double max_ver_acc_;    // [m/s]
  double max_attitude_;   // [rad]
  double max_head_rate_;  // [rad/s]

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::Accel> accel_pub_;
  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;

  void getStaticRosParams(tobas::BaseNode* node);

  void publishAccel(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& acc);
  void publishAngle(const builtin_interfaces::msg::Time& stamp, const kdl::Euler& angle);
};
}  // namespace tobas_rc_teleop
