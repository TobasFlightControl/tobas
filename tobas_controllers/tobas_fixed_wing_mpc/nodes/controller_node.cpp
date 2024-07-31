#include "../include/tobas_fixed_wing_mpc/controller.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "tobas_fixed_wing_mpc");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  tobas_fixed_wing_mpc::Controller node(node, pnh);
  rclcpp::spin();
}
