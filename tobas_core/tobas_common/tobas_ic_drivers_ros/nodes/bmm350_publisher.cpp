// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tobas_ic_drivers/bmm350.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std::chrono_literals;

namespace tobas
{
class Bmm350PublisherNode : public rclcpp::Node
{
public:
  explicit Bmm350PublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  bool initialize();

private:
  static bool toOdr(int _odr_hz, uint8_t& _odr);
  static bool toAveraging(int _averaging, uint8_t& _odr);
  void timerCallback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  driver::BMM350 mag_;
  double mx_, my_, mz_;  // micro tesla
  bool initialized_ = false;
};

/**
 * @note 起動例
 * ros2 run tobas_ic_drivers bmm350_publisher --ros-args -p odr_hz=:100 -p averaging:4
 * odr_hz: 25 or 100, averaging: 2 or 4 それぞれ電流が３倍になる．
 */
Bmm350PublisherNode::Bmm350PublisherNode(const rclcpp::NodeOptions& options) : Node("bmm350_publisher", options)
{
  this->declare_parameter<int>("odr_hz", 100);
  this->declare_parameter<int>("averaging", 4);

  int odr_hz = this->get_parameter("odr_hz").as_int();
  uint8_t odr;
  if (!toOdr(odr_hz, odr) || (odr_hz <= 0) || ((1000 % odr_hz) != 0)) {
    RCLCPP_WARN(this->get_logger(), "Invald odr_hz parameter. Falling back to 100Hz publish timer.");
    odr_hz = 100;
  }
  const std::chrono::milliseconds publish_period(1000 / odr_hz);  // ms
  publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("magnetic_field", 1);
  timer_ = this->create_wall_timer(publish_period, std::bind(&Bmm350PublisherNode::timerCallback, this));
}

bool Bmm350PublisherNode::toOdr(int odr_hz, uint8_t& odr)
{
  if (odr_hz == 100) {
    odr = driver::BMM350::ODR_100Hz;
    return true;
  }

  if (odr_hz == 25) {
    odr = driver::BMM350::ODR_25Hz;
    return true;
  }

  return false;
}

bool Bmm350PublisherNode::toAveraging(int averaging, uint8_t& avg)
{
  if (averaging == 2) {
    avg = driver::BMM350::AVG_2;
    return true;
  }

  if (averaging == 4) {
    avg = driver::BMM350::AVG_4;
    return true;
  }

  return false;
}

bool Bmm350PublisherNode::initialize()
{
  const int odr_hz = this->get_parameter("odr_hz").as_int();
  const int averaging = this->get_parameter("averaging").as_int();

  uint8_t odr = 0;
  uint8_t avg = 0;

  if (!toOdr(odr_hz, odr) || !toAveraging(averaging, avg)) {
    RCLCPP_WARN(this->get_logger(), "Invalid parameters. odr_hz is must be 25 or 100, averaging must be 2 or 4.");
    return false;
  }

  if (!mag_.configure(odr, avg)) {
    RCLCPP_WARN(this->get_logger(), "Failed to stage BMM350 configuration.");
    return false;
  }
  if (!mag_.initialize()) {
    RCLCPP_WARN(this->get_logger(), "Failed to initialize magnetometer.");
    return false;
  }
  return true;
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
