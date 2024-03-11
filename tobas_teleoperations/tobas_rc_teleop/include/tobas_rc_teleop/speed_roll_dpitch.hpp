#pragma once

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class SpeedRollDeltaPitchController : public BaseController
{
  static constexpr char kControllerName[] = "speed_roll_dpitch_controller";

  static constexpr double kDefaultMinSpeed = 5.;           // [m/s]
  static constexpr double kDefaultMaxSpeed = 20.;          // [m/s]
  static constexpr double kDefaultMaxRoll = M_PI_2;        // [rad]
  static constexpr double kDefaultMaxDeltaPitch = M_PI_4;  // [rad]

  using super = BaseController;

public:
  explicit SpeedRollDeltaPitchController(const tobas::Drone& drone);

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(
    const tobas_msgs::RCInput& rcin,
    const tobas_msgs::Odometry& odom,
    const double& battery_voltage) override;

private:
  // rosparams
  double min_speed_;   // [m/s]
  double max_speed_;   // [m/s]
  double max_roll_;    // [rad]
  double max_dpitch_;  // [rad]

  // PubSub
  ros::Publisher cmd_pub_;

  void getRosParams(ros::NodeHandle& pnh);
  void registerPublishers(ros::NodeHandle& nh);
};
}  // namespace tobas_rc_teleop
