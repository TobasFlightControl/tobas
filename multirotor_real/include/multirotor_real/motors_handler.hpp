#pragma once

#include <ros/ros.h>
#include <mav_msgs/Actuators.h>

#include <Navio2/RCOutput_Navio2.h>

#include <multirotor_tools/rotor_property.hpp>

class MotorsHandler
{
public:
  MotorsHandler(ros::NodeHandle& nh);

private:
  const uint32_t num_rotors_;
  const RotorProperties rotor_props_;
  RCOutput_Navio2 pwm_;

  ros::Subscriber rotor_vels_sub_;

  void rotorVelsCb(const mav_msgs::Actuators& rotor_vels);
};
