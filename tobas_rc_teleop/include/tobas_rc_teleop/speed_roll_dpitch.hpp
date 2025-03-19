#pragma once

#include <tobas_command_msgs/msg/speed_roll_delta_pitch.hpp>

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
  explicit SpeedRollDeltaPitchController();

  bool requirePosition() override;
  bool requireOrientation() override;
  bool requireLinearVelocity() override;
  bool requireAngularVelocity() override;

  void initialize(tobas::BaseNode* node) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  // rosparams
  double min_speed_;   // [m/s]
  double max_speed_;   // [m/s]
  double max_roll_;    // [rad]
  double max_dpitch_;  // [rad]

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::msg::SpeedRollDeltaPitch> cmd_pub_;

  void getStaticRosParams(tobas::BaseNode* node);
};
}  // namespace tobas_rc_teleop
