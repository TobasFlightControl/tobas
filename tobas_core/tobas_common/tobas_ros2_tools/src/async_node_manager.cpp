#include "tobas_ros2_tools/async_node_manager.hpp"

namespace ros2
{
AsyncNodeManager::AsyncNodeManager(int argc, char** argv, const std::string& node_name)
{
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }

  node_ = rclcpp::Node::make_shared(node_name);
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });
}

AsyncNodeManager::~AsyncNodeManager()
{
  executor_->cancel();
  executor_thread_->join();
}

rclcpp::Node::SharedPtr AsyncNodeManager::node()
{
  return node_;
}

rclcpp::Node::ConstSharedPtr AsyncNodeManager::node() const
{
  return node_;
}
}  // namespace ros2
