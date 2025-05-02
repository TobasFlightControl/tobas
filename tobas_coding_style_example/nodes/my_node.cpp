#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/string.hpp>

#include "tobas_coding_style_example/my_class.hpp"

using namespace std::chrono_literals;

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
