#include "../include/tobas_property_tools/property_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "property_server");
  rclcpp::NodeHandle node;
  rclcpp::NodeHandle pnh("~");
  ptree::PropertyServer node(node, pnh);
  rclcpp::spin();
}
