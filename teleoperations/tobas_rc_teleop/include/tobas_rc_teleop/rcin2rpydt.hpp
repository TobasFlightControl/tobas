#pragma once

#include <dh_std_tools/math.hpp>

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
  static constexpr double kDefaultMaxYawrate = dh_std::deg2rad(90.);

  using super = tobas::BaseNode;

public:
  explicit RcinToRollPitchYawrateThrust();

private:
  tobas::Drone drone_;
  tobas::RotorAxisExtractor z_rotors_;

  bool battery_received_;
  tobas_msgs::Battery battery_;
  tobas_msgs::RollPitchYawrateThrust rpydt_;

  // rosparams
  double max_attitude_;  // [rad]
  double max_yawrate_;   // [rad/s]

  // PubSub
  ros::Publisher rpydt_pub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber rcin_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::Event& event) override;
  void batteryCb(const tobas_msgs::Battery& battery);
  void rcInputCb(const tobas_msgs::RCInput& rcin);
};
}  // namespace tobas_rc_teleop
