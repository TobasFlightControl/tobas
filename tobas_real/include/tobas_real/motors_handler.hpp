#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <Navio2/RCOutput_Navio2.h>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/Battery.h>

namespace tobas_real
{
class MotorsHandler : public tobas::BaseNode
{
  static constexpr double kCheckIntervalRate = 10.;      // [Hz]
  static constexpr double kAutoStopTimeThreshold = 0.5;  // [s]
  static constexpr double kThrottleMargin = 0.01;

  using super = tobas::BaseNode;

public:
  explicit MotorsHandler(ros::NodeHandle nh, ros::NodeHandle pnh);

private:
  tobas::Drone drone_;
  RCOutput_Navio2 pwm_;

  ros::Time last_cmd_time_;  // [s]
  bool is_activated_;
  bool battery_received_;
  tobas_msgs::Battery battery_;

  // PubSub
  ros::Subscriber rotor_speeds_sub_;
  ros::Subscriber battery_sub_;

  // Timer
  ros::Timer check_interval_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady();
  void sendDisarm();
  void setPeriodOnAllChannels(double period);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void rotorSpeedsCb(const tobas_msgs::RotorSpeeds& speeds);
  void batteryCb(const tobas_msgs::Battery& battery);

  void checkIntervalTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_real
