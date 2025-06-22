#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tobas_ic_drivers/bmm150.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

class Bmm150PublisherNode : public rclcpp::Node
{
public:
  explicit Bmm150PublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  bool initialize();

private:
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  driver::BMM150 mag_;
  double mx_, my_, mz_;  // micro tesla
  bool initialized_ = false;
};
