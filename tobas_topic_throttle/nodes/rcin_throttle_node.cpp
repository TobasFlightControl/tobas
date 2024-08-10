#include "../include/tobas_topic_throttle/rcin_throttle.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "rcin_throttle");
  rclcpp::Node::SharedPtr node;
  tobas_topic_throttle::RCInputThrottle node(node);
  rclcpp::spin();
}
