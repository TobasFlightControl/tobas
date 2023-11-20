#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <Navio2/RCOutput_Navio2.h>

#include <dh_std_tools/first_order_filter.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/Battery.h>

namespace tobas_real
{
class MotorsHandler : public tobas::BaseNode
{
  static constexpr size_t kCheckIntervalRate = 10;       // [Hz]
  static constexpr double kAutoStopTimeThreshold = 0.5;  // [s]
  static constexpr double kThrottleMargin = 0.01;

  using self = MotorsHandler;
  using super = tobas::BaseNode;

public:
  explicit MotorsHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;
  RCOutput_Navio2 pwm_;

  ros::Time last_cmd_time_;  // [s]
  bool is_activated_ = false;
  tobas_msgs::BatteryConstPtr battery_;
  dh_std::FirstOrderFilter<double> latency_filter_;

  // PubSub
  ros::Publisher rotor_speeds_pub_;
  ros::Subscriber rotor_speeds_sub_;
  ros::Subscriber battery_sub_;

  // Timer
  ros::Timer check_interval_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void sendDisarm();
  void setPeriodOnAllChannels(const double& period);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& speeds);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);

  void checkIntervalTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_real
