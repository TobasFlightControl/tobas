#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

#include <tobas_tools/node.hpp>

namespace tobas_navio_ros
{
class TimeReferenceServer : public tobas::BaseNode
{
  static constexpr size_t kUpdateRate = 1;  // [Hz]

  using self = TimeReferenceServer;
  using super = tobas::BaseNode;

public:
  explicit TimeReferenceServer(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  ros::Publisher time_ref_pub_;
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros
