#pragma once

#include <ros/ros.h>

#include <Navio2/DSHOT.h>

#include <dh_ros_tools/node.hpp>

#include <tobas_tools/rotor_property.hpp>
#include <tobas_msgs/RotorSpeeds.h>

class MotorsHandler_DSHOT : public dh_ros::BaseNode
{
  const double kDefaultUpdateRate = 1000.;

public:
  MotorsHandler_DSHOT();
  void run();

private:
  bool cmd_received_;
  std::vector<double> cmd_speeds_;
  DSHOT dshot_;

  // rosparams
  std::string drone_name_;
  double battery_voltage_;
  int num_rotors_;
  RotorConfigs rotor_configs_;
  double update_rate_;

  // PubSub
  ros::Subscriber rotor_vels_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;
  void createTimers() override;

  void rotorSpeedsCb(const tobas_msgs::RotorSpeeds& rotor_speeds);
  void checkTopicsTimerCb(const ros::TimerEvent& event) override;
};
