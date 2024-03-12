#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <std_srvs/SetBool.h>

#include <tobas_std_tools/first_order_filter.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/Battery.h>

namespace tobas_real
{
class MotorsHandler : public tobas::BaseNode
{
  static constexpr double kBLHeliClosedLoopLowRangeMaxERPM = 50000;
  static constexpr double kBLHeliClosedLoopMidRangeMaxERPM = 100000;
  static constexpr double kBLHeliClosedLoopHighRangeMaxERPM = 200000;

  static constexpr size_t kCheckIntervalTimerRate = 10;  // [Hz]
  static constexpr double kSetupPwmRetryInterval = 1.;   // [s]
  static constexpr bool kDefaultBlockBelowArmSpeed = true;

  using self = MotorsHandler;
  using super = tobas::BaseNode;

public:
  explicit MotorsHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;

  ros::Time last_cmd_time_;
  bool is_armed_ = false;
  bool is_activated_ = false;
  tobas_msgs::BatteryConstPtr battery_;
  tobas_std::FirstOrderFilter<double> latency_filter_;

  // rosparams
  bool block_below_arm_speed_;

  // PubSub
  ros::Publisher pwms_pub_;
  ros::Publisher cur_speeds_pub_;
  ros::Publisher latency_pub_;
  ros::Subscriber tar_speeds_sub_;
  ros::Subscriber battery_sub_;

  // Service
  ros::ServiceServer arm_rotors_ss_;
  ros::ServiceClient setup_pwm_sc_;
  ros::ServiceClient enable_pwm_sc_;

  // Timer
  ros::Timer setup_pwm_timer_;
  ros::Timer check_interval_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool armRotors();
  bool disarmRotors();
  bool enablePwms(const bool& enable);
  void setPeriodOnAllChannels(const double& period);

  void rotSpeedsCmdCb(const tobas_msgs::RotorSpeedsConstPtr& tar_speeds);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);

  bool armRotorsCb(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);

  void setupPwmTimerCb(const ros::TimerEvent& event);
  void checkIntervalTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_real
