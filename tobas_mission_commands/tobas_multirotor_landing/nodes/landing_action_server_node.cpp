#include "../include/tobas_multirotor_landing/landing_action_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "landing_action_server");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_multirotor_landing::LandActionServer node(node, pnh);
  rclcpp::spin();
}
