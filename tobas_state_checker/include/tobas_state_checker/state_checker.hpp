#pragma once

#include <rclcpp/rclcpp.hpp>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_kdl_msgs/EulerStamped.h>
#include <tobas_msgs/Event.h>
#include <tobas_msgs/Cpu.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/LandAction.h>

namespace tobas_state_checker
{
class StateChecker : public tobas::BaseNode
{
  static constexpr double kWarnPeriod = 3.;                            // [s]
  static constexpr double kWaitForActionServerTimeout = 3.;            // [s]
  static constexpr double kCpuTempertureThreshold = 80.;               // [celsius]
  static constexpr double kAttitudeThreshold = 85. * tobas::kDeg2Rad;  // [rad]

  using self = StateChecker;
  using super = tobas::BaseNode;

public:
  explicit StateChecker(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr pnh, const std::string& name = rclcpp::this_node::getName());

private:
  tobas::Drone drone_;
  std_msgs::BoolConstPtr arming_;

  // Publishers
  rclcpp::Publisher event_pub_;

  // Subscribers
  rclcpp::Subscriber arming_sub_;
  rclcpp::Subscriber cpu_sub_;
  rclcpp::Subscriber battery_sub_;
  rclcpp::Subscriber euler_sub_;

  rclcpp::ServiceClient set_arm_sc_;
  actionlib::SimpleActionClient<tobas_msgs::LandAction> landing_ac_;

  void publishSystemCriticalEvent();
  void requestLanding();
  void requestDisarmingRotors();

  void armingCb(const std_msgs::BoolConstPtr& arming);
  void cpuCb(const tobas_msgs::CpuConstPtr& cpu);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void eulerCb(const tobas_kdl_msgs::EulerStampedConstPtr& euler);
};
}  // namespace tobas_state_checker
