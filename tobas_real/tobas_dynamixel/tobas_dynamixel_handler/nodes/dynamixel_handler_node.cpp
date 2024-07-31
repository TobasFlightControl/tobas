#include "../include/tobas_dynamixel_handler/dynamixel_handler.hpp"

using namespace std;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "dynamixel_handler");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_dynamixel_handler::DynamixelHandler node(node, pnh);
  rclcpp::spin();
}
