#pragma once

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PoseTwistAccelController : public BaseController
{
  using super = BaseController;

public:
  explicit PoseTwistAccelController(const tobas::Drone& drone);

  void initialize(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr pnh) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void
  update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, const double& battery_voltage) override;

private:
  // rosparams
  double max_hor_vel_;   // [m/s]
  double max_ver_vel_;   // [m/s]
  double max_attitude_;  // [rad]
  double max_yawrate_;   // [rad/s]

  // Mutable
  bool is_up_commanded_;
  rclcpp::Time t_last_rcin_;
  kdl::Vector tar_vel_F_;  // フットプリント座標系から見た目標速度
  kdl::Vector tar_pos_W_;  // 世界座標系から見た目標位置
  kdl::Euler tar_rpy_;

  // Publisher
  rclcpp::Publisher cmd_pub_;

  void getRosParams(rclcpp::Node::SharedPtr pnh);
};
}  // namespace tobas_rc_teleop
