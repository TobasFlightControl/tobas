#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ros/timer.h>

#include <tobas_node/node.hpp>

namespace tobas_real_ros
{
class TimeReferenceServer : public tobas::BaseNode
{
  static constexpr size_t kUpdateRate = 1;  // [Hz]

  using self = TimeReferenceServer;
  using super = tobas::BaseNode;

public:
  explicit TimeReferenceServer(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  PublisherPtr<> time_ref_pub_;
  rclcpp::Timer main_timer_;

  void mainTimerCb(const rclcpp::TimerEvent& event);
};
}  // namespace tobas_real_ros
