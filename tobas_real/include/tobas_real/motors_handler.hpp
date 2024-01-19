#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <Navio2/PWM.h>

#include <tobas_std_tools/first_order_filter.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/Throttles.h>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/Battery.h>

namespace tobas_real
{
class MotorsHandler : public tobas::BaseNode
{
  static constexpr size_t kSetupPwmTimerRate = 1;        // [Hz]
  static constexpr size_t kDisarmTimerRate = 10;         // [Hz]
  static constexpr size_t kCheckIntervalTimerRate = 10;  // [Hz]
  static constexpr double kThrottleMargin = 1e-3;

  using self = MotorsHandler;
  using super = tobas::BaseNode;

public:
  explicit MotorsHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;
  PWM pwm_;

  ros::Time disarm_start_time_;
  ros::Time last_cmd_time_;
  bool is_activated_ = false;
  tobas_msgs::BatteryConstPtr battery_;
  tobas_std::FirstOrderFilter<double> latency_filter_;

  // PubSub
  ros::Publisher rotor_speeds_pub_;
  ros::Subscriber throttles_sub_;
  ros::Subscriber battery_sub_;

  // Timer
  ros::Timer setup_pwm_timer_;
  ros::Timer disarm_timer_;
  ros::Timer check_interval_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void setPeriodOnAllChannels(const double& period);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void throttlesCmdCb(const tobas_msgs::ThrottlesConstPtr& throttles);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);

  void setupPwmTimerCb(const ros::TimerEvent& event);
  void disarmTimerCb(const ros::TimerEvent& event);
  void checkIntervalTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_real
