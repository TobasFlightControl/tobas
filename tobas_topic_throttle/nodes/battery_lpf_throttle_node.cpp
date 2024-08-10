#include "../include/tobas_topic_throttle/battery_lpf_throttle.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "battery_lpf_throttle");
  rclcpp::Node::SharedPtr node;
  tobas_topic_throttle::BatteryLPFThrottle node(node);
  rclcpp::spin();
}
