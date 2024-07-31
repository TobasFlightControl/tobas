#include "../include/tobas_rotor_controller/rotor_controller.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "rotor_controller");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_rotor_controller::RotorController node(node, pnh);
  rclcpp::spin();
}
