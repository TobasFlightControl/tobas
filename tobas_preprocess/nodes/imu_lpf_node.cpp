#include "../include/tobas_preprocess/imu_lpf.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "imu_lpf");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_preprocess::ImuLpf node(node, pnh);
  rclcpp::spin();
}
