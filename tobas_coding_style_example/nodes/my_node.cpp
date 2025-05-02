#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/string.hpp>

namespace tobas
{
class MyNode : public rclcpp::Node
{
public:
  explicit MyNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

MyNode::MyNode(const rclcpp::NodeOptions& options) : rclcpp::Node("my_node", options)
{
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::MyNode)
