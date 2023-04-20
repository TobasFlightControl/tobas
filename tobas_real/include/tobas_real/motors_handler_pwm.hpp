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

  // rosparam
  const double battery_voltage_;
  const uint32_t num_rotors_;
  const RotorConfigs rotor_configs_;

  RCOutput_Navio2 pwm_;

  ros::Subscriber rotor_vels_sub_;

  uint32_t getChannel(uint32_t pin);
  void rotorSpeedsCb(const tobas_msgs::RotorSpeeds& speeds);
};
