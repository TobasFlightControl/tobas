#pragma once

#include <tobas_tools/node.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/RCInput.h>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class RCTeleop : public tobas::BaseNode
{
  static constexpr double kInitThrustThreshold = 0.05;

  using self = RCTeleop;
  using super = tobas::BaseNode;

public:
  explicit RCTeleop(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  enum stage_t
  {
    CHECK_PREREQUISITES,
    WAIT_FOR_ESTOP,
    ESTOP_ON,
    FIRST_COMMAND,
    RUNNING,
    DISARMED,
  } stage_ = CHECK_PREREQUISITES;

  tobas::Drone drone_;

  // rosparams
  std::array<std::string, tobas::kNumFlightModes> modes_;

  // Mutables
  uint8_t last_mode_;
  tobas_msgs::OdometryConstPtr odom_;
  tobas_msgs::BatteryConstPtr battery_;

  // Controllers
  std::array<std::unique_ptr<BaseController>, tobas::kNumFlightModes> controllers_;

  // PubSub
  ros::Subscriber odom_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber rcin_sub_;

  // Service
  ros::ServiceClient get_arm_sc_;
  ros::ServiceClient set_arm_sc_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isRotorsArmed();
  bool requestArmingRotors();
  bool requestDisarmingRotors();

  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void rcInputCb(const tobas_msgs::RCInputConstPtr& rcin);
};
}  // namespace tobas_rc_teleop
