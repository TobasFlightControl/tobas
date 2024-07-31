#include "../include/tobas_latency_publisher/latency_publisher.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "latency_publisher");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_latency_publisher::LatencyPublisher node(node, pnh);
  rclcpp::spin();
}
