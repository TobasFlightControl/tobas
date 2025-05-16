#pragma once

#include <tobas_ros2_tools/register.hpp>

#include <tobas_msgs/msg/latency.hpp>

namespace tobas
{
class ImuSamplingTimePublisher
{
public:
  explicit ImuSamplingTimePublisher();

  void initialize(rclcpp::Node::SharedPtr node, const rclcpp::Time& cur_time);
  void publish(const rclcpp::Time& cur_time);

private:
  rclcpp::Time last_time_;
  ros2::PublisherPtr<tobas_msgs::msg::Latency> pub_;
};
}  // namespace tobas
