#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ros2
{
/**
 * @brief メインスレッドとは別のスレッドで動作するROSノードを作成，管理する．
 */
class AsyncNodeManager
{
public:
  explicit AsyncNodeManager(int argc, char** argv, const std::string& node_name);

  rclcpp::Node::SharedPtr node();
  rclcpp::Node::ConstSharedPtr node() const;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::shared_ptr<std::thread> executor_thread_;
};
}  // namespace ros2
