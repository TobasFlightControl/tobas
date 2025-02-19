#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

class UserNode : public rclcpp::Node
{
public:
  explicit UserNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
};

UserNode::UserNode(const rclcpp::NodeOptions& options) : rclcpp::Node("user_cpp_node", options)
{
}

RCLCPP_COMPONENTS_REGISTER_NODE(UserNode)
