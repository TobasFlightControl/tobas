#include "../include/tobas_mr_mpc/controller_ros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "tobas_mr_mpc");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_mr_mpc::ControllerRos node(node, pnh);
  rclcpp::spin();
}
