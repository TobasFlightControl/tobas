#include "tobas_ic_drivers_ros/bmm150_publisher.hpp"

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

Bmm150PublisherNode::Bmm150PublisherNode(const rclcpp::NodeOptions& options) : Node("bmm150_publisher", options)
{
  publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("magnetic_field", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&Bmm150PublisherNode::timerCallback, this));
}

bool Bmm150PublisherNode::initialize()
{
  if (!mag_.initialize()) {
    RCLCPP_WARN(this->get_logger(), "Failed to initialize magnetometer.");
    return false;
  }
  return true;
}

void Bmm150PublisherNode::timerCallback()
{
  if (!initialized_) {
    initialized_ = initialize();
  }
  if (!mag_.readMag(mx_, my_, mz_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to read magnetic field.");
    return;
  }
  auto message =  std::make_unique<geometry_msgs::msg::PointStamped>();
  message->header.frame_id = "map";
  message->point.x = mx_;
  message->point.y = my_;
  message->point.z = mz_;
  RCLCPP_INFO(this->get_logger(), "Publishing: '%lf, %lf, %lf' [μT]", mx_, my_, mz_);
  publisher_->publish(std::move(message));
}

RCLCPP_COMPONENTS_REGISTER_NODE(Bmm150PublisherNode)
