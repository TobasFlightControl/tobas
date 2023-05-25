#pragma once

#include <ros/ros.h>

#include <Navio2/DSHOT.h>

#include <dh_ros_tools/node.hpp>
#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/drone.hpp>
#include <tobas_msgs/RotorSpeeds.h>

namespace tobas_real
{
class MotorsHandler_DSHOT : public dh_ros::BaseNode
{
  const double kDefaultUpdateRate = 1000.;

  using super = dh_ros::BaseNode;

public:
  explicit MotorsHandler_DSHOT();
  void run();

private:
  tobas::Drone drone_;
  DSHOT dshot_;

  bool is_initialized_;
  bool cmd_received_;
  std::vector<double> cmd_speeds_;

  // rosparams
  double update_rate_;

  // PubSub
  ros::Subscriber rotor_vels_sub_;

  // Timer
  dh_ros::Timer check_topics_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void rotorSpeedsCb(const tobas_msgs::RotorSpeeds& rotor_speeds);
  void checkTopicsTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_real
