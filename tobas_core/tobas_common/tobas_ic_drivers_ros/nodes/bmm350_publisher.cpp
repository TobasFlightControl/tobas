// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

// Usage: ros2 run tobas_ic_drivers bmm350_publisher --ros-args -p odr_hz=:100 -p averaging:4
// Param: odr_hz: 25 or 100, averaging: 2 or 4

#include <chrono>
#include <expected>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tobas_ic_drivers/bmm350.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std::chrono_literals;
namespace ch = std::chrono;

namespace tobas
{
class Bmm350PublisherNode : public rclcpp::Node
{
public:
  explicit Bmm350PublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  bool initialize();

private:
  std::expected<driver::BMM350::ODR, const char*> toOdr(int odr_hz);
  std::expected<driver::BMM350::Averaging, const char*> toAveraging(int averaging);
  void timerCallback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  driver::BMM350 mag_;
  float mx_, my_, mz_;  // micro tesla
  bool initialized_ = false;
};

Bmm350PublisherNode::Bmm350PublisherNode(const rclcpp::NodeOptions& options) : Node("bmm350_publisher", options)
{
  this->declare_parameter<int>("odr_hz", 100);
  this->declare_parameter<int>("averaging", 4);

  int odr_hz = this->get_parameter("odr_hz").as_int();
  const auto odr = toOdr(odr_hz);
  if (!odr) {
    RCLCPP_WARN(this->get_logger(), "%s", odr.error());
    throw std::runtime_error("Invalid odr_hz parameter");
  }
  const ch::milliseconds publish_period(1000 / odr_hz);  // ms

  publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("magnetic_field", 1);
  timer_ = this->create_wall_timer(publish_period, std::bind(&Bmm350PublisherNode::timerCallback, this));
}

bool Bmm350PublisherNode::initialize()
{
  const int odr_hz = this->get_parameter("odr_hz").as_int();
  const int averaging = this->get_parameter("averaging").as_int();
  const auto avg = toAveraging(averaging);
  const auto odr = toOdr(odr_hz);

  if (!avg) {
    RCLCPP_WARN(this->get_logger(), "%s", avg.error());
    return false;
  }
  if (!odr) {
    RCLCPP_WARN(this->get_logger(), "%s", odr.error());
    return false;
  }
  if (!mag_.configure(odr.value(), avg.value())) {
    RCLCPP_WARN(this->get_logger(), "Failed to stage BMM350 configuration.");
    return false;
  }
  if (!mag_.initialize()) {
    RCLCPP_WARN(this->get_logger(), "Failed to initialize magnetometer.");
    return false;
  }

  return true;
}

std::expected<driver::BMM350::ODR, const char*> Bmm350PublisherNode::toOdr(int odr_hz)
{
  if (odr_hz == 100) {
    return driver::BMM350::ODR_100Hz;
  }
  if (odr_hz == 25) {
    return driver::BMM350::ODR_25Hz;
  }
  return std::unexpected("invalid ODR");
}

std::expected<driver::BMM350::Averaging, const char*> Bmm350PublisherNode::toAveraging(int averaging)
{
  if (averaging == 2) {
    return driver::BMM350::AVG_2;
  }
  if (averaging == 4) {
    return driver::BMM350::AVG_4;
  }
  return std::unexpected("invalid averaging");
}

void Bmm350PublisherNode::timerCallback()
{
  if (!initialized_) {
    initialized_ = initialize();
    return;
  }
  if (!mag_.readMag(mx_, my_, mz_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to read magnetic field.");
    return;
  }
  auto message = std::make_unique<geometry_msgs::msg::PointStamped>();
  message->header.frame_id = "map";
  message->point.x = mx_;
  message->point.y = my_;
  message->point.z = mz_;
  RCLCPP_INFO(this->get_logger(), "Publishing: '%lf, %lf, %lf' [μT]", mx_, my_, mz_);
  publisher_->publish(std::move(message));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::Bmm350PublisherNode)
