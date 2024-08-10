#include "../include/tobas_multirotor_move/move_action_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "move_action_server");
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr pnh("~");
  tobas_multirotor_move::MoveActionServer node(node, pnh);
  rclcpp::spin();
}
