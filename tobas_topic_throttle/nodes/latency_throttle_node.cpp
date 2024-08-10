#include "../include/tobas_topic_throttle/latency_throttle.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "latency_throttle");
  rclcpp::Node::SharedPtr node;
  tobas_topic_throttle::LatencyThrottle node(node);
  rclcpp::spin();
}
