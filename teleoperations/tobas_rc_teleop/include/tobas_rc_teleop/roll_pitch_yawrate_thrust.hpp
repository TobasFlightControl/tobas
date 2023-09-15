#pragma once

#include <dh_std_tools/range.hpp>

#include <tobas_tools/drone.hpp>
#include <tobas_tools/rotor_axis_extractor.hpp>
#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/RCInput.h>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class RollPitchYawrateThrustController : public BaseController
{
  // Constants
  static constexpr char kControllerName[] = "roll_pitch_yawrate_thrust_controller";

  // Default parameters
  static constexpr double kDefaultMaxAttitude = M_PI / 6;  // [rad]
  static constexpr double kDefaultMaxYawrate = M_PI;       // [rad/s]
  static constexpr double kDefaultMaxAcceleration = 3.;    // [m/s^2]
  static constexpr double kDefaultMinAcceleration = -3.;   // [m/s^2]

  using super = BaseController;

public:
  explicit RollPitchYawrateThrustController();

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;
  void reset();
  void update(
    const tobas_msgs::RCInput& rcin,
    const double& battery_voltage,
    const dh_std::Range<double>& dead_zone);

private:
  tobas::Drone drone_;
  tobas::RotorAxisExtractor z_rotors_;

  // rosparams
  double max_attitude_;  // [rad]
  double max_yawrate_;   // [rad/s]
  double max_acc_;       // [m/s^2] 垂直上方向の加速度の最大値
  double min_acc_;       // [m/s^2] 垂直下方向の加速度の最大値

  // Constant values
  double max_thrust_;  // [N] ドローンの最大合計推力
  double min_thrust_;  // [N] ドローンの最小合計推力

  // PubSub
  ros::Publisher rpydt_pub_;

  void getRosParams(ros::NodeHandle& pnh);
  void registerPublishers(ros::NodeHandle& nh);
};
}  // namespace tobas_rc_teleop
