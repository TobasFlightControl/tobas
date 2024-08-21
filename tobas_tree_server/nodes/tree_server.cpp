#include <std_msgs/msg/string.hpp>

#include <tobas_std_tools/debug.hpp>
#include <tobas_kdl/kdl_parser.hpp>
#include <tobas_kdl_msgs/Tree.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

using namespace std;

class TreeServerNode : public tobas::BaseNode
{
  using self = TreeServerNode;
  using super = tobas::BaseNode;

public:
  explicit TreeServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<kdl::Tree> tree_pub_;
  ros2::SubscriberPtr<std_msgs::msg::String> description_sub_;

  void descriptionCb(const std_msgs::msg::String::ConstSharedPtr& msg);
};

TreeServerNode::TreeServerNode(const rclcpp::NodeOptions& options) : super("tree_server", options)
{
  tree_pub_ = createPublisher<kdl::Tree>(tobas::kKDLTreeTopic, true);
  description_sub_ = createSubscriber(tobas::kRobotDescriptionTopic, &self::descriptionCb, this, true);
}

void TreeServerNode::descriptionCb(const std_msgs::msg::String::ConstSharedPtr& msg)
{
  auto tree = std::make_unique<kdl::Tree>();

  if (!kdl::treeFromString(msg->data, *tree))
  {
    TOBAS_ERROR("Failed to parse robot description.");
    return;
  }

  tree_pub_->publish(move(tree));
}

RCLCPP_COMPONENTS_REGISTER_NODE(TreeServerNode)
