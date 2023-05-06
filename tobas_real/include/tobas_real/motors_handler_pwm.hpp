#pragma once

#include <ros/ros.h>

#include <Navio2/RCOutput_Navio2.h>

#include <dh_ros_tools/node.hpp>

#include <tobas_tools/rotor_property.hpp>
#include <tobas_msgs/RotorSpeeds.h>

class MotorsHandler_PWM : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

public:
  explicit MotorsHandler_PWM();

private:
  RCOutput_Navio2 pwm_;

  // rosparam
  std::string drone_name_;
  double battery_voltage_;
  int num_rotors_;
  RotorConfigs rotor_configs_;

  // PubSub
  ros::Subscriber rotor_vels_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;
  void createTimers() override;

  uint32_t getChannel(uint32_t pin);

  void rotorSpeedsCb(const tobas_msgs::RotorSpeeds& speeds);
  void checkTopicsTimerCb(const ros::TimerEvent& event) override;
};
