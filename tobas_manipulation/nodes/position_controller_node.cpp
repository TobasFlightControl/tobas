#include "../include/tobas_manipulation/position_controller_ros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "tobas_manipulation_position");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_manipulation::PositionControllerRos node(node, pnh);
  rclcpp::spin();
}
