#pragma once

#include <rclcpp/rclcpp.hpp>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/msg/bool.hpp>

#include <tobas_node/node.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_kdl_msgs/EulerStamped.hpp>
#include <tobas_msgs/Event.h>
#include <tobas_msgs/Cpu.h>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/LandAction.h>

namespace tobas_state_checker
{
class StateChecker : public tobas::BaseNode
{
  static constexpr double kWarnPeriod = 3.;                            // [s]
  static constexpr double kWaitForActionServerTimeout = 3.;            // [s]
  static constexpr double kCpuTempertureThreshold = 80.;               // [celsius]
  static constexpr double kAttitudeThreshold = 85. * tobas_std::kDeg2Rad;  // [rad]

  using self = StateChecker;
  using super = tobas::BaseNode;

public:
  explicit StateChecker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;
  std_msgs::msg::Bool::ConstSharedPtr arming_;

  // Publishers
  PublisherPtr<> event_pub_;

  // Subscribers
  SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  SubscriberPtr<> cpu_sub_;
  SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  SubscriberPtr<> euler_sub_;

  rclcpp::ServiceClient set_arm_sc_;
  actionlib::SimpleActionClient<tobas_msgs::LandAction> landing_ac_;

  void publishSystemCriticalEvent();
  void requestLanding();
  void requestDisarmingRotors();

  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void cpuCb(const tobas_msgs::Cpu::ConstSharedPtr& cpu);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void eulerCb(const tobas_kdl_msgs::EulerStamped::ConstSharedPtr& euler);
};
}  // namespace tobas_state_checker
