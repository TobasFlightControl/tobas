#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std;

class DummyNode : public rclcpp::Node
{
public:
  explicit DummyNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

DummyNode::DummyNode(const rclcpp::NodeOptions& options) : rclcpp::Node("dummy_node", options)
{
  RCLCPP_WARN_STREAM(get_logger(), "Dummy \"" << get_name() << "\" has started.");
}

RCLCPP_COMPONENTS_REGISTER_NODE(DummyNode)
