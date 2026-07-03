// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "./future.hpp"

/* For development. */
// #include <tobas_std_msgs/action/empty.hpp>
// using ActType = tobas_std_msgs::action::Empty;

namespace tobas
{
namespace ros2
{
/**
 * @brief Synchronous action client.
 * @note This blocks, so do not use it in nodes where real-time behavior is important.
 */
template <typename ActType>
class SyncActionClient
{
  static constexpr auto kWaitForServer = std::chrono::seconds(1);

  using Client = rclcpp_action::Client<ActType>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActType>;
  using GoalHandlePtr = typename GoalHandle::SharedPtr;
  using FeedbackCb = std::function<void(const GoalHandlePtr&, const typename ActType::Feedback::ConstSharedPtr&)>;

public:
  using SharedPtr = std::shared_ptr<SyncActionClient>;

  explicit SyncActionClient(
    rclcpp::Node::SharedPtr node,
    const std::string& name,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
    : node_(node), action_name_(name)
  {
    client_ = rclcpp_action::create_client<ActType>(node, name, group);
  }

  /**
   * @brief Call the action.
   *
   * @param goal Action goal.
   *
   * @note Calling this from a callback that runs on the same thread as the ROS node causes a deadlock.
   */
  std::pair<GoalHandlePtr, std::shared_future<typename GoalHandle::WrappedResult>>
  sendGoal(const typename ActType::Goal& goal, FeedbackCb feedback_cb = nullptr)
  {
    if (!client_->wait_for_action_server(kWaitForServer)) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << action_name_ << "\" action server is not ready.");
      return {};
    }

    typename Client::SendGoalOptions opts;
    opts.feedback_callback = feedback_cb;

    const auto send_goal_future = client_->async_send_goal(goal, opts);
    send_goal_future.wait();

    const auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Goal was rejected by \"" << action_name_ << "\" action server.");
      return {};
    }

    return { goal_handle, client_->async_get_result(goal_handle) };
  }

  /**
   * @brief Call the action and wait until a result is obtained.
   *
   * @param goal Action goal.
   * @param get_result_timeout,cancel_goal_timeout Timeout for each step. Waits indefinitely when non-positive.
   *
   * @note Calling this from a callback that runs on the same thread as the ROS node causes a deadlock.
   */
  bool sendGoalAndWait(
    const typename ActType::Goal& goal,
    FeedbackCb feedback_cb = nullptr,
    std::chrono::milliseconds get_result_timeout = std::chrono::milliseconds(-1),
    std::chrono::milliseconds cancel_goal_timeout = std::chrono::milliseconds(-1))
  {
    const auto [goal_handle, get_result_future] = sendGoal(goal, feedback_cb);
    if (!get_result_future.valid()) {
      return false;
    }

    if (waitForFuture(get_result_future, get_result_timeout) != std::future_status::ready) {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(), "Timeout before getting \"" << action_name_ << "\" action result. Cancelling goal...");

      if (!cancelGoalAndWait(goal_handle, cancel_goal_timeout)) {
        return false;
      }

      return false;
    }

    result_ = get_result_future.get();

    return true;
  }

  std::shared_future<std::shared_ptr<action_msgs::srv::CancelGoal_Response>> cancelGoal(GoalHandlePtr goal_handle)
  {
    return client_->async_cancel_goal(goal_handle);
  }

  bool cancelGoalAndWait(GoalHandlePtr goal_handle, std::chrono::milliseconds timeout = std::chrono::milliseconds(-1))
  {
    const auto cancel_goal_future = client_->async_cancel_goal(goal_handle);
    if (waitForFuture(cancel_goal_future, timeout) != std::future_status::ready) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to cancel \"" << action_name_ << "\" action goal.");
      return false;
    }
    return true;
  }

  inline const typename GoalHandle::WrappedResult& getResult() const
  {
    return result_;
  }

private:
  const rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  typename Client::SharedPtr client_;
  typename GoalHandle::WrappedResult result_;
};
}  // namespace ros2
}  // namespace tobas
