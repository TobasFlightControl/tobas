#pragma once

#include <tobas_command_msgs_adapter/accel.hpp>
#include <tobas_command_msgs_adapter/rate.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class AccelRateController : public BaseController
{
  using super = BaseController;

public:
  explicit AccelRateController();

  bool requirePosition() override;
  bool requireOrientation() override;
  bool requireLinearVelocity() override;
  bool requireAngularVelocity() override;

  void initialize(tobas::BaseNode* node) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  kdl::Vector tar_acc_G_;
  kdl::Vector tar_gyro_B_;

  // rosparams
  double max_hor_acc_;    // [m/s]
  double max_ver_acc_;    // [m/s]
  double max_atti_rate_;  // [rad/s]
  double max_head_rate_;  // [rad/s]

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::Accel> accel_pub_;
  ros2::PublisherPtr<tobas_command_msgs::Rate> rate_pub_;

  void getStaticRosParams(tobas::BaseNode* node);

  void publishAccel(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& acc);
  void publishRate(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& rate);
};
}  // namespace tobas_rc_teleop
