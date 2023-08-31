#pragma once

#include <ros/ros.h>
#include <Navio2/RCOutput_Navio2.h>

#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/Battery.h>

namespace tobas_real
{
class MotorsHandler_PWM : public tobas::BaseNode
{
  static constexpr double kControlRate = 800.;           // [Hz]
  static constexpr double kAutoStopTimeThreshold = 0.5;  // [s]
  static constexpr double kThrottleMargin = 0.01;

  using super = tobas::BaseNode;

public:
  explicit MotorsHandler_PWM(ros::NodeHandle nh, ros::NodeHandle pnh);

  void run();

private:
  tobas::Drone drone_;
  RCOutput_Navio2 pwm_;

  ros::Time last_cmd_time_;  // [s]
  bool is_activated_;
  bool is_initialized_;
  bool battery_received_;
  std::vector<double> pwm_periods_;
  tobas_msgs::Battery battery_;

  // PubSub
  ros::Subscriber rotor_speeds_sub_;
  ros::Subscriber battery_sub_;

  // Timer
  dh_ros::Timer check_topics_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady();
  void sendDisarm();

  void eventCb(const tobas_msgs::Event& event) override;
  void rotorSpeedsCb(const tobas_msgs::RotorSpeeds& speeds);
  void batteryCb(const tobas_msgs::Battery& battery);

  void checkTopicsTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_real
