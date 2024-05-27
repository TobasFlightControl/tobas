#pragma once

#include <tobas_tools/rotor_axis_extractor.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class RollPitchYawThrustController : public BaseController
{
  using super = BaseController;

public:
  explicit RollPitchYawThrustController(const tobas::Drone& drone);

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void
  update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, const double& battery_voltage) override;

private:
  tobas::RotorAxisExtractor z_rotors_;

  double yaw_;
  ros::Time t_last_rcin_;

  // rosparams
  double max_attitude_;  // [rad]
  double max_yawrate_;   // [rad/s]
  double max_ver_acc_;   // [m/s^2] 垂直方向の加速度の最大値

  // Constant values
  double max_thrust_;  // [N] ドローンの最大推力和

  // PubSub
  ros::Publisher rpy_thrust_pub_;

  void getRosParams(ros::NodeHandle& pnh);
};
}  // namespace tobas_rc_teleop
