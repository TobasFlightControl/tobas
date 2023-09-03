#pragma once

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/range.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_tools/rotor_axis_extractor.hpp>
#include <tobas_msgs/RollPitchYawrateThrust.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/RCInput.h>

namespace tobas_rc_teleop
{
class RcinToRollPitchYawrateThrust : public tobas::BaseNode
{
  static constexpr double kDefaultMaxAttitude = dh_std::deg2rad(30.);
  static constexpr double kDefaultMaxYawrate = dh_std::deg2rad(180.);
  static constexpr double kDefaultMaxAcceleration = 3.;
  static constexpr double kDefaultMinAcceleration = -3.;

  using super = tobas::BaseNode;

public:
  explicit RcinToRollPitchYawrateThrust(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  enum Stage
  {
    CHECK_PREREQUISITES,
    FIRST_RCIN,
    TOGGLE_OFF,
    TOGGLE_ON,
  };

  tobas::Drone drone_;
  tobas::RotorAxisExtractor z_rotors_;

  Stage stage_ = CHECK_PREREQUISITES;
  tobas_msgs::BatteryConstPtr battery_;

  // ROS parameters
  double max_attitude_;  // [rad]
  double max_yawrate_;   // [rad/s]
  double max_acc_;       // [m/s^2] 垂直上方向の加速度の最大値
  double min_acc_;       // [m/s^2] 垂直下方向の加速度の最大値
  double dead_zone_rate_;

  // Constant values
  double max_thrust_;  // [N] ドローンの最大合計推力
  double min_thrust_;  // [N] ドローンの最小合計推力
  dh_std::Range<double> dead_zone_;

  // PubSub
  ros::Publisher rpydt_pub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber rcin_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void rcInputCb(const tobas_msgs::RCInputConstPtr& rcin);
};
}  // namespace tobas_rc_teleop
