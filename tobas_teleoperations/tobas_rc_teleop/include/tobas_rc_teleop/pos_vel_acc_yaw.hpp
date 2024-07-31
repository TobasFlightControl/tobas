#pragma once

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PosVelAccYawController : public BaseController
{
  using super = BaseController;

public:
  explicit PosVelAccYawController(const tobas::Drone& drone);

  void initialize(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr pnh) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void
  update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, const double& battery_voltage) override;

private:
  bool is_up_commanded_;
  rclcpp::Time t_last_rcin_;
  kdl::Vector tar_vel_F_;
  kdl::Vector tar_pos_W_;
  double tar_yaw_;

  // rosparams
  double max_hor_vel_;  // [m/s]
  double max_ver_vel_;  // [m/s]
  double max_yawrate_;  // [rad/s]

  // Publisher
  rclcpp::Publisher cmd_pub_;

  void getRosParams(rclcpp::Node::SharedPtr pnh);
};
}  // namespace tobas_rc_teleop
