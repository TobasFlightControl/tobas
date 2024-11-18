#include <std_msgs/msg/string.hpp>

#include <tobas_std_tools/debug.hpp>
#include <tobas_kdl/kdl_parser.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
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
  ros2::TimerPtr initialize_timer_;

  void initializeTimerCb();
  void descriptionCb(const std_msgs::msg::String::ConstSharedPtr& msg);
};

TreeServerNode::TreeServerNode(const rclcpp::NodeOptions& options) : super("tree_server", options)
{
  // 起動時に既に発行済のURDFを確実に取得するために，Pubscriberの登録を遅延させる．
  initialize_timer_ = createTimer(0ns, &self::initializeTimerCb, this);
}

void TreeServerNode::initializeTimerCb()
{
  tree_pub_ = createPublisher<kdl::Tree>(tobas::kKDLTreeTopic, true, true);
  description_sub_ = createSubscriber(tobas::kRobotDescriptionTopic, &self::descriptionCb, this, true, true);

  initialize_timer_->cancel();
}

void TreeServerNode::descriptionCb(const std_msgs::msg::String::ConstSharedPtr& msg)
{
  TOBAS_INFO("New robot description is received.");

  auto tree = std::make_unique<kdl::Tree>();
  if (!kdl::treeFromString(msg->data, *tree))
  {
    TOBAS_ERROR("Failed to parse robot description.");
    return;
  }

  tree_pub_->publish(move(tree));
}

RCLCPP_COMPONENTS_REGISTER_NODE(TreeServerNode)
