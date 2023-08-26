#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <dh_std_tools/math.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Cpu.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/LandAction.h>

namespace tobas_state_checker
{
static constexpr double kUpdateRate = 10.;                               // [Hz]
static constexpr double kWarnPeriod = 3.;                                // [s]
static constexpr double kWaitForActionServer = 3.;                       // [s]
static constexpr double kWarnCpuTemperature = 70.;                       // [degree celsius]
static constexpr double kFatalCpuTemperture = 80.;                       // [degree celsius]
static constexpr double kHorizontalPositionStddevThreshold = 1.;         // [m]
static constexpr double kVerticalPositionStddevThreshold = 2.;           // [m]
static constexpr double kAttitudeStddevThreshold = dh_std::deg2rad(5.);  // [rad]
static constexpr double kHeadingStddevThreshold = dh_std::deg2rad(10.);  // [rad]
static constexpr double kBaseStateTimeout = 0.5;                         // [s]
static constexpr double kAttitudeThreshold = dh_std::deg2rad(90.);       // [rad]
static constexpr double kBatteryVoltageWarnTime = 3.;                    // [s]
static constexpr double kBatteryVoltageFatalTime = 60.;                  // [s]

class MultirotorStateChecker : public tobas::BaseNode
{
  using super = tobas::BaseNode;

public:
  explicit MultirotorStateChecker();

  void run();

private:
  // rosparams
  double voltage_threshold_;  // 飛行を継続できる電圧の閾値

  bool battery_received_;
  bool bs_received_;
  bool cmd_received_;
  ros::Time t_last_valid_voltage_;
  ros::Time t_last_bs_;

  ros::Subscriber cpu_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber bs_sub_;
  ros::Subscriber cmd_sub_;

  actionlib::SimpleActionClient<tobas_msgs::LandAction> ac_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void requestLanding();

  void eventCb(const tobas_msgs::Event& event) override;
  void cpuCb(const tobas_msgs::Cpu& cpu);
  void batteryCb(const tobas_msgs::Battery& battery);
  void baseStateCb(const tobas_msgs::BaseState& bs);
};
}  // namespace tobas_state_checker
