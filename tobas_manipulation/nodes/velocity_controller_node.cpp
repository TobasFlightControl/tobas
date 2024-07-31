#include "../include/tobas_manipulation/velocity_controller_ros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "tobas_manipulation_velocity");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_manipulation::VelocityControllerRos node(node, pnh);
  rclcpp::spin();
}
