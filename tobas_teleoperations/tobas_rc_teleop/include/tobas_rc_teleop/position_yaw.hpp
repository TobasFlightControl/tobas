#pragma once

#include <tobas_msgs/PositionYaw.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PositionYawController : public BaseController
{
  using super = BaseController;

public:
  explicit PositionYawController();

  void initialize(tobas::BaseNode* node) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry& odom)
    override;

private:
  bool is_up_commanded_ = false;
  tobas_msgs::PositionYaw pos_yaw_;
  kdl::Vector vel_;
  builtin_interfaces::msg::Time t_last_rcin_;

  // rosparams
  double max_hor_vel_;  // [m/s]
  double max_ver_vel_;  // [m/s]
  double max_yawrate_;  // [rad/s]

  // Publisher
  ros2::PublisherPtr<tobas_msgs::PositionYaw> pos_yaw_pub_;

  void getStaticRosParams(tobas::BaseNode* node);
};
}  // namespace tobas_rc_teleop
