#pragma once

#include <ros/ros.h>

#include <Navio2/RCOutput_Navio2.h>

#include <dh_ros_tools/node.hpp>
#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/drone.hpp>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/Battery.h>

namespace tobas_real
{
class MotorsHandler_PWM : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

public:
  explicit MotorsHandler_PWM();

private:
  tobas::Drone drone_;
  RCOutput_Navio2 pwm_;

  bool is_initialized_;
  bool rot_speeds_received_;
  bool battery_received_;
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
  uint32_t getChannel(uint32_t pin);

  void rotorSpeedsCb(const tobas_msgs::RotorSpeeds& speeds);
  void batteryCb(const tobas_msgs::Battery& battery);

  void checkTopicsTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_real
