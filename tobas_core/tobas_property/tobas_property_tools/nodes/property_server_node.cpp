#include "../include/tobas_property_tools/property_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ptree::PropertyServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
