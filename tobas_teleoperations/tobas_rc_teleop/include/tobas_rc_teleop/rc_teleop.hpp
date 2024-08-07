#pragma once

#include <map>

#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/RCInput.h>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class RCTeleop : public tobas::BaseNode
{
  static constexpr double kInitThrottleMargin = 0.05;
  static constexpr double kArmFailRetryInterval = 1.;  // [s]

  using self = RCTeleop;
  using super = tobas::BaseNode;

public:
  explicit RCTeleop(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum stage_t
  {
    CHECK_PREREQUISITES,
    WAIT_FOR_ESTOP,
    ESTOP_ON,
    FIRST_COMMAND,
    RUNNING,
  } stage_ = CHECK_PREREQUISITES;

  const std::map<uint8_t, const char*> mode2str_{
    { tobas::kFlightModeProgram, "Program" },
    { tobas::kFlightModeStabilize, "Stabilize" },
    { tobas::kFlightModeAcrobat, "Acrobat" },
  };

  tobas::Drone drone_;

  // rosparams
  std::array<std::string, tobas::kNumFlightModes> modes_;

  // Mutables
  uint8_t last_mode_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;

  // Controllers
  std::array<std::unique_ptr<BaseController>, tobas::kNumFlightModes> controllers_;

  // PubSub
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  SubscriberPtr<> rcin_sub_;

  // Service
  rclcpp::ServiceClient get_arm_sc_;
  rclcpp::ServiceClient set_arm_sc_;

  void getRosParams();
  bool isRotorsArmed();
  bool requestArmingRotors();
  bool requestDisarmingRotors();

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);
};
}  // namespace tobas_rc_teleop
