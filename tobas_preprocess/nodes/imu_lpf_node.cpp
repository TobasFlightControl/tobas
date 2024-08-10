#include "../include/tobas_preprocess/imu_lpf.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "imu_lpf");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_preprocess::ImuLpf node(node, pnh);
  rclcpp::spin();
}
