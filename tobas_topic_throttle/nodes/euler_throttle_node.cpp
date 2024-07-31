#include "../include/tobas_topic_throttle/euler_throttle.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "euler_throttle");
  rclcpp::NodeHandle node;
  tobas_topic_throttle::EulerThrottle node(node);
  rclcpp::spin();
}
