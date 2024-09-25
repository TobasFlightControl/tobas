#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "./future.hpp"

/* 開発用 */
// #include <tobas_std_msgs/action/empty.hpp>
// using ActionType = tobas_std_msgs::action::Empty;

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
   * @brief アクションを呼ぶ．
   *
   * @param goal アクションゴール．
   *
   * @note ROSノードと同じスレッドで動作するコールバックの中で呼ぶとデッドロックする．
   */
  std::pair<
    std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionType>>,
    std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult>>
  sendGoal(const typename ActionType::Goal& goal)
  {
    if (!client_->action_server_is_ready())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << action_name_ << "\" action server is not ready.");
      return {};
    }

    auto send_goal_future = client_->async_send_goal(goal);
    send_goal_future.wait();

    const auto goal_handle = send_goal_future.get();
    if (goal_handle == nullptr)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Goal was rejected by \"" << action_name_ << "\" action server.");
      return {};
    }

    return { goal_handle, client_->async_get_result(goal_handle) };
  }

  /**
   * @brief アクションを呼び，結果が得られるまで待機する．
   *
   * @param goal アクションゴール．
   * @param get_result_timeout,cancel_goal_timeout 該当ステップのタイムアウト．非正ならば無限待機．
   *
   * @note ROSノードと同じスレッドで動作するコールバックの中で呼ぶとデッドロックする．
   */
  bool sendGoalAndWait(
    const typename ActionType::Goal& goal,
    std::chrono::milliseconds get_result_timeout = std::chrono::milliseconds(-1),
    std::chrono::milliseconds cancel_goal_timeout = std::chrono::milliseconds(-1))
  {
    auto [goal_handle, get_result_future] = sendGoal(goal);
    if (!get_result_future.valid())
      return false;

    if (waitForFuture(get_result_future, get_result_timeout) != std::future_status::ready)
    {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(), "Timeout before getting \"" << action_name_ << "\" action result. Cancelling goal...");

      if (!cancelGoalAndWait(goal_handle, cancel_goal_timeout))
        return false;

      return false;
    }

    result_ = get_result_future.get();

    return true;
  }

  std::shared_future<std::shared_ptr<action_msgs::srv::CancelGoal_Response>>
  cancelGoal(std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionType>> goal_handle)
  {
    return client_->async_cancel_goal(goal_handle);
  }

  bool cancelGoalAndWait(
    std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionType>> goal_handle,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(-1))
  {
    auto cancel_goal_future = client_->async_cancel_goal(goal_handle);
    if (waitForFuture(cancel_goal_future, timeout) != std::future_status::ready)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to cancel \"" << action_name_ << "\" action goal.");
      return false;
    }
    return true;
  }

  inline const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& getResult() const
  {
    return result_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  typename rclcpp_action::Client<ActionType>::SharedPtr client_;
  typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult result_;
};
}  // namespace ros2
