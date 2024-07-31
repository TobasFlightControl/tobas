#include "../include/tobas_manipulation/effort_controller_ros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "tobas_manipulation_effort");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_manipulation::EffortControllerRos node(node, pnh);
  rclcpp::spin();
}
