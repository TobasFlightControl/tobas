#include "../include/tobas_preprocess/battery_lpf.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "battery_lpf");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_preprocess::BatteryLpf node(node, pnh);
  rclcpp::spin();
}
