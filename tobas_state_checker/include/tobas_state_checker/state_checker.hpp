#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Cpu.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/LandAction.h>

namespace tobas_state_checker
{
static constexpr double kUpdateRate = 10.;                        // [Hz]
static constexpr double kWarnPeriod = 3.;                         // [s]
static constexpr double kWaitForActionServer = 10.;               // [s]
static constexpr double kWarnCpuTemperature = 70.;                // [degree celsius]
static constexpr double kFatalCpuTemperture = 80.;                // [degree celsius]
static constexpr double kPoseTwistTimeout = 0.5;                  // [s]
static constexpr double kAttitudeThreshold = M_PI_2;              // [rad]
static constexpr double kBatteryVoltageWarnTime = 3.;             // [s]
static constexpr double kBatteryVoltageFatalTime = 60.;           // [s]

class StateChecker : public tobas::BaseNode
{
  using super = tobas::BaseNode;

public:
  explicit StateChecker(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

  void run();

private:
  // rosparams
  double voltage_threshold_;  // 飛行を継続できる電圧の閾値

  bool pt_received_ = false;
  ros::Time t_last_valid_voltage_;
  ros::Time t_last_pt_;

  ros::Subscriber cpu_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber pt_sub_;
  ros::Subscriber cmd_sub_;

  actionlib::SimpleActionClient<tobas_msgs::LandAction> ac_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void requestLanding();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void cpuCb(const tobas_msgs::CpuConstPtr& cpu);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt);
};
}  // namespace tobas_state_checker
