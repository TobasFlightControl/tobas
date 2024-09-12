#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "./future.hpp"

namespace ros2
{
template <typename ActionType>
class SyncActionClient
{
public:
  using SharedPtr = std::shared_ptr<SyncActionClient>;

  inline explicit SyncActionClient(
    rclcpp::Node::SharedPtr node,
    const std::string& name,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
    : node_(node), action_name_(name)
  {
    client_ = rclcpp_action::create_client<ActionType>(node, name, group);
  }

  /**
   * @brief アクションを呼び，結果が得られるまで待機する．
   *
   * @param goal アクションゴール．
   * @param get_result_timeout,send_goal_timeout,cancel_goal_timeout 該当ステップのタイムアウト．非正ならば無限待機．
   *
   * @note ROSノードと同じスレッドで動作するコールバックの中で呼ぶとデッドロックする．
   */
  bool sendGoalAndWait(
    const ActionType::Goal& goal,
    std::chrono::milliseconds get_result_timeout = std::chrono::milliseconds(-1),
    std::chrono::milliseconds send_goal_timeout = std::chrono::milliseconds(-1),
    std::chrono::milliseconds cancel_goal_timeout = std::chrono::milliseconds(-1))
  {
    if (!client_->action_server_is_ready())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << action_name_ << "\" action server is not ready.");
      return false;
    }

    auto send_goal_future = client_->async_send_goal(goal);
    if (waitForFuture(send_goal_future, send_goal_timeout) != std::future_status::ready)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Timeout before sending \"" << action_name_ << "\" action goal.");
      return false;
    }

    const auto goal_handle = send_goal_future.get();
    if (goal_handle == nullptr)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Goal was rejected by \"" << action_name_ << "\" action server.");
      return false;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting for \"" << action_name_ << "\" action result...");
    auto get_result_future = client_->async_get_result(goal_handle);
    if (waitForFuture(get_result_future, get_result_timeout) != std::future_status::ready)
    {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(), "Timeout before getting \"" << action_name_ << "\" action result. Cancelling goal...");

      auto cancel_goal_future = client_->async_cancel_goal(goal_handle);
      if (waitForFuture(cancel_goal_future, cancel_goal_timeout) != std::future_status::ready)
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to cancel \"" << action_name_ << "\" action goal.");

      return false;
    }

    result_ = get_result_future.get();

    return true;
  }

  inline const rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& getResult() const
  {
    return result_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  rclcpp_action::Client<ActionType>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult result_;
};
}  // namespace ros2
