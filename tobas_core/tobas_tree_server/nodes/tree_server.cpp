#include <tobas_constants/ros_interface.hpp>
#include <tobas_kdl_parser/kdl_parser.hpp>
#include <tobas_node/node.hpp>

#include <std_msgs/msg/string.hpp>

#include <tobas_kdl_msgs_adapter/tree.hpp>

using namespace std::chrono_literals;

class TreeServerNode : public tobas::BaseNode
{
  using self = TreeServerNode;
  using super = tobas::BaseNode;

public:
  explicit TreeServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  kdl::Tree tree_;
  kdl::TreeParser tree_parser_;

  ros2::PublisherPtr<kdl::Tree> tree_pub_;
  ros2::SubscriberPtr<std_msgs::msg::String> description_sub_;
  ros2::TimerPtr initialize_timer_;

  void publishTree();

  void initializeTimerCb();
  void descriptionCb(const std_msgs::msg::String::ConstSharedPtr& msg);
};

TreeServerNode::TreeServerNode(const rclcpp::NodeOptions& options) : super("tree_server", nodeOptions_Default(options))
{
  // 起動時に既に発行済のURDFを確実に取得するために，Pubscriberの登録を遅延させる．
  initialize_timer_ = createTimer(0s, &self::initializeTimerCb, this);
}

void TreeServerNode::publishTree()
{
  auto tree_msg = std::make_unique<kdl::Tree>(tree_);
  tree_pub_->publish(std::move(tree_msg));
}

void TreeServerNode::initializeTimerCb()
{
  tree_pub_ = createPublisher<kdl::Tree>(tobas::topic::kKdlTree, true, true);
  description_sub_ = createSubscriber(tobas::topic::kRobotDescription, &self::descriptionCb, this, true, true);

  initialize_timer_->cancel();
}

void TreeServerNode::descriptionCb(const std_msgs::msg::String::ConstSharedPtr& msg)
{
  TOBAS_INFO("New robot description is received.");

  if (!tree_parser_.parseFromText(msg->data, tree_)) {
    TOBAS_ERROR("Failed to parse robot description: ", tree_parser_.errorMessage());
    return;
  }

  std::string error_msg;
  if (!tree_.isValid(error_msg)) {
    TOBAS_ERROR("KDL tree is invalid: ", error_msg);
    return;
  }

  publishTree();
}

RCLCPP_COMPONENTS_REGISTER_NODE(TreeServerNode)
