#include "../include/tobas_fixed_wing_lqd/controller.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "tobas_fixed_wing_lqd");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_fixed_wing_lqd::Controller node(node, pnh);
  rclcpp::spin();
}
