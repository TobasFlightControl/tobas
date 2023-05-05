#pragma once

#include <ros/ros.h>

#include <Navio2/RCOutput_Navio2.h>

#include <tobas_tools/rotor_property.hpp>
#include <tobas_msgs/RotorSpeeds.h>

class MotorsHandler_PWM
{
public:
  MotorsHandler_PWM();

private:
  ros::NodeHandle nh_;

  RCOutput_Navio2 pwm_;

  // rosparam
  std::string drone_name_;
  double battery_voltage_;
  int num_rotors_;
  RotorConfigs rotor_configs_;

  // PubSub
  ros::Subscriber rotor_vels_sub_;

  void getRosParams();
  void registerSubscribers();
  uint32_t getChannel(uint32_t pin);

  void rotorSpeedsCb(const tobas_msgs::RotorSpeeds& speeds);
};
