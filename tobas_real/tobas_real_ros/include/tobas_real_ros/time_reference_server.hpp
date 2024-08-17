#pragma once

#include <rclcpp/rclcpp.hpp>


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
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  PublisherPtr<> time_ref_pub_;
  rclcpp::Timer main_timer_;

  void mainTimerCb(const rclcpp::TimerEvent& event);
};
}  // namespace tobas_real_ros
