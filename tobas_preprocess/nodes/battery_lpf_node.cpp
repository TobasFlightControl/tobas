#include "../include/tobas_preprocess/battery_lpf.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "battery_lpf");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_preprocess::BatteryLpf node(node, pnh);
  rclcpp::spin();
}
