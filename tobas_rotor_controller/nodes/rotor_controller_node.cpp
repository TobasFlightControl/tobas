#include "../include/tobas_rotor_controller/rotor_controller.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "rotor_controller");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_rotor_controller::RotorController node(node, pnh);
  rclcpp::spin();
}
