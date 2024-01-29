#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Cpu.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/LandAction.h>

namespace tobas_state_checker
{
class StateChecker : public tobas::BaseNode
{
  static constexpr double kWarnPeriod = 3.;                  // [s]
  static constexpr double kWaitForActionServerTimeout = 3.;  // [s]
  static constexpr double kCpuTempertureThreshold = 70.;     // [degree celsius]
  static constexpr double kAttitudeThreshold = M_PI_2;       // [rad]

  using self = StateChecker;
  using super = tobas::BaseNode;

public:
  explicit StateChecker(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  // rosparams
  double voltage_threshold_;  // 飛行を継続できる電圧の閾値

  // PubSub
  ros::Subscriber cpu_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cmd_sub_;

  actionlib::SimpleActionClient<tobas_msgs::LandAction> landing_ac_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void requestLanding();

  void cpuCb(const tobas_msgs::CpuConstPtr& cpu);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
};
}  // namespace tobas_state_checker
