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

  using self = SpeedRollDeltaPitchController;
  using super = BaseController;

public:
  explicit SpeedRollDeltaPitchController();

  bool requirePosition() override;
  bool requireOrientation() override;
  bool requireLinearVelocity() override;
  bool requireAngularVelocity() override;

  void initialize(tobas::BaseNode* node, tobas::flight_mode_t mode) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  // rosparams
  double min_speed_ = 0.;                                  // [m/s]
  double max_speed_ = std::numeric_limits<double>::max();  // [m/s]
  double max_roll_;                                        // [rad]
  double max_dpitch_;                                      // [rad]

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::msg::SpeedRollDeltaPitch> cmd_pub_;

  bool minSpeedCb(const double& p);
  bool maxSpeedCb(const double& p);
  bool maxRollCb(const double& p);
  bool maxDeltaPitchCb(const double& p);
};
}  // namespace tobas_rc_teleop
