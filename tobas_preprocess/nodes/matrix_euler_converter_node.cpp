#include "../include/tobas_preprocess/matrix_euler_converter.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "matrix_euler_converter");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_preprocess::MatrixEulerConverter node(node, pnh);
  rclcpp::spin();
}
