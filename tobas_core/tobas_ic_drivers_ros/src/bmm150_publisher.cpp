#include "tobas_ic_drivers_ros/bmm150_publisher.hpp"

using namespace std::chrono_literals;

Bmm150Publisher::Bmm150Publisher() : Node("bmm150_publisher")
{
  publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("magnetic_field", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&Bmm150Publisher::timerCallback, this));
}

bool Bmm150Publisher::initialize()
{
  if (!mag_.initialize()) {
    RCLCPP_WARN(this->get_logger(), "Failed to initialize magnetometer.");
    return false;
  }
  return true;
}

void Bmm150Publisher::timerCallback()
{
  if (!initialized_) {
    initialize();
    initialized_ = true;
  }
  if (!mag_.readMag(mx_, my_, mz_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to read magnetic field.");
    return;
  }
  auto message = geometry_msgs::msg::PointStamped();
  message.header.frame_id = "map";
  message.point.x = mx_;
  message.point.y = my_;
  message.point.z = mz_;
  RCLCPP_INFO(this->get_logger(), "Publishing: '%lf, %lf, %lf' [μT]", mx_, my_, mz_);
  publisher_->publish(message);
}
