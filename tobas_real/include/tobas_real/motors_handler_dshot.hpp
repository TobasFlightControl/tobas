#pragma once

#include <ros/ros.h>

#include <Navio2/DSHOT.h>

#include <tobas_tools/rotor_property.hpp>
#include <tobas_msgs/RotorSpeeds.h>

class MotorsHandler_DSHOT
{
  const double kDefaultUpdateRate = 1000.;

public:
  MotorsHandler_DSHOT();
  void run();

private:
  ros::NodeHandle nh_;

  const double battery_voltage_;
  const uint32_t num_rotors_;
  const RotorConfigs rotor_configs_;
  const double update_rate_;

  std::vector<double> cmd_speeds_;
  DSHOT dshot_;

  ros::Subscriber rotor_vels_sub_;

  void rotorSpeedsCb(const tobas_msgs::RotorSpeeds& rotor_speeds);
};
