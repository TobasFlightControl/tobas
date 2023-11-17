#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <Navio2/RCOutput_Navio2.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Throttles.h>

namespace tobas_motor_test
{
class MotorsHandler : public tobas::BaseNode
{
  static constexpr uint32_t kUpdateRate = 100;  // [Hz]
  static constexpr double kWarnPeriod = 3.;     // [s]

  using self = MotorsHandler;
  using super = tobas::BaseNode;

public:
  explicit MotorsHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  RCOutput_Navio2 pwm_;
  tobas_msgs::ThrottlesConstPtr throttles_;

  ros::Subscriber throttles_sub_;
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void sendDisarm();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void throttlesCb(const tobas_msgs::ThrottlesConstPtr& throttles);

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_motor_test
