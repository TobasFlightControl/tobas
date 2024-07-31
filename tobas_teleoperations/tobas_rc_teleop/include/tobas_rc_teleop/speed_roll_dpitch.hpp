#pragma once

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class SpeedRollDeltaPitchController : public BaseController
{
  static constexpr double kDefaultMinSpeed = 5.;           // [m/s]
  static constexpr double kDefaultMaxSpeed = 20.;          // [m/s]
  static constexpr double kDefaultMaxRoll = M_PI_2;        // [rad]
  static constexpr double kDefaultMaxDeltaPitch = M_PI_4;  // [rad]

  using super = BaseController;

public:
  explicit SpeedRollDeltaPitchController(const tobas::Drone& drone);

  void initialize(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr pnh) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void
  update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, const double& battery_voltage) override;

private:
  // rosparams
  double min_speed_;   // [m/s]
  double max_speed_;   // [m/s]
  double max_roll_;    // [rad]
  double max_dpitch_;  // [rad]

  // PubSub
  rclcpp::Publisher cmd_pub_;

  void getRosParams(rclcpp::Node::SharedPtr pnh);
};
}  // namespace tobas_rc_teleop
