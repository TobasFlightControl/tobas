// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>

#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>

#include <action_msgs/srv/cancel_goal.hpp>

#include "./future.hpp"

/* For development */
// #include <tobas_std_msgs/action/empty.hpp>
// using ActType = tobas_std_msgs::action::Empty;

namespace tobas
{
namespace ros2
{
/**
 * Synchronous action client.
 *
 * @note Action calls block the calling thread. Do not use this client where real-time behavior is required.
 */
template <typename ActType>
class SyncActionClient
{
  using Client = rclcpp_action::Client<ActType>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActType>;
  using GoalHandlePtr = typename GoalHandle::SharedPtr;
  using Result = typename GoalHandle::WrappedResult;
  using FeedbackCb = std::function<void(const GoalHandlePtr&, const typename ActType::Feedback::ConstSharedPtr&)>;

public:
  explicit SyncActionClient(
    rclcpp::Node::SharedPtr _node,
    const std::string& _name,
    rclcpp::CallbackGroup::SharedPtr _group = nullptr);

  /**
   * Call the action.
   *
   * @param _goal Action goal.
   *
   * @note Calling this from a callback that runs on the same thread as the ROS node causes a deadlock.
   */
  std::pair<GoalHandlePtr, std::shared_future<Result>>
  sendGoal(const typename ActType::Goal& _goal, FeedbackCb _feedback_cb = nullptr);

  /**
   * Call the action and wait until a result is obtained.
   *
   * @param _goal Action goal.
   * @param _get_result_timeout,cancel_goal_timeout Timeout for each step. Waits indefinitely when non-positive.
   *
   * @note Calling this from a callback that runs on the same thread as the ROS node causes a deadlock.
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  std::optional<Result> sendGoalAndWait(
    const typename ActType::Goal& _goal,
    FeedbackCb _feedback_cb = nullptr,
    std::chrono::duration<RepT, RatioT> _get_result_timeout = std::chrono::duration<RepT, RatioT>(-1),
    std::chrono::duration<RepT, RatioT> _cancel_goal_timeout = std::chrono::duration<RepT, RatioT>(-1));

  std::shared_future<std::shared_ptr<action_msgs::srv::CancelGoal_Response>> cancelGoal(GoalHandlePtr _goal_handle);

  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool cancelGoalAndWait(
    GoalHandlePtr _goal_handle,
    std::chrono::duration<RepT, RatioT> _timeout = std::chrono::duration<RepT, RatioT>(-1));

  inline bool serverIsReady() const;

  template <typename RepT = int64_t, typename RatioT = std::milli>
  inline bool waitForServer(std::chrono::duration<RepT, RatioT> _timeout = std::chrono::duration<RepT, RatioT>(-1));

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string action_name_;
  typename Client::SharedPtr client_;
};

template <typename ActType>
SyncActionClient<ActType>::SyncActionClient(
  rclcpp::Node::SharedPtr _node,
  const std::string& _name,
  rclcpp::CallbackGroup::SharedPtr _group)
  : node_(_node), action_name_(_name)
{
  client_ = rclcpp_action::create_client<ActType>(_node, _name, _group);
}

template <typename ActType>
std::pair<typename SyncActionClient<ActType>::GoalHandlePtr, std::shared_future<typename SyncActionClient<ActType>::Result>>
SyncActionClient<ActType>::sendGoal(const typename ActType::Goal& _goal, FeedbackCb _feedback_cb)
{
  if (!client_->action_server_is_ready()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "'" << action_name_ << "' action server is not ready.");
    return {};
  }

  typename Client::SendGoalOptions opts;
  opts.feedback_callback = _feedback_cb;

  const auto send_goal_future = client_->async_send_goal(_goal, opts);
  send_goal_future.wait();

  const auto goal_handle = send_goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Goal was rejected by '" << action_name_ << "' action server.");
    return {};
  }

  return { goal_handle, client_->async_get_result(goal_handle) };
}

template <typename ActType>
template <typename RepT, typename RatioT>
std::optional<typename SyncActionClient<ActType>::Result> SyncActionClient<ActType>::sendGoalAndWait(
  const typename ActType::Goal& _goal,
  FeedbackCb _feedback_cb,
  std::chrono::duration<RepT, RatioT> _get_result_timeout,
  std::chrono::duration<RepT, RatioT> _cancel_goal_timeout)
{
  const auto [goal_handle, get_result_future] = sendGoal(_goal, _feedback_cb);
  if (!get_result_future.valid()) {
    return std::nullopt;
  }

  if (waitForFuture(get_result_future, _get_result_timeout) != std::future_status::ready) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(), "Timeout before getting '" << action_name_ << "' action result. Cancelling goal...");

    if (!cancelGoalAndWait(goal_handle, _cancel_goal_timeout)) {
      return std::nullopt;
    }

    return std::nullopt;
  }

  return get_result_future.get();
}

template <typename ActType>
std::shared_future<std::shared_ptr<action_msgs::srv::CancelGoal_Response>>
SyncActionClient<ActType>::cancelGoal(GoalHandlePtr _goal_handle)
{
  return client_->async_cancel_goal(_goal_handle);
}

template <typename ActType>
template <typename RepT, typename RatioT>
bool SyncActionClient<ActType>::cancelGoalAndWait(
  GoalHandlePtr _goal_handle,
  std::chrono::duration<RepT, RatioT> _timeout)
{
  const auto cancel_goal_future = client_->async_cancel_goal(_goal_handle);
  if (waitForFuture(cancel_goal_future, _timeout) != std::future_status::ready) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to cancel '" << action_name_ << "' action goal.");
    return false;
  }
  return true;
}

template <typename ActType>
inline bool SyncActionClient<ActType>::serverIsReady() const
{
  return client_->action_server_is_ready();
}

template <typename ActType>
template <typename RepT, typename RatioT>
inline bool SyncActionClient<ActType>::waitForServer(std::chrono::duration<RepT, RatioT> _timeout)
{
  return client_->wait_for_action_server(_timeout);
}
}  // namespace ros2
}  // namespace tobas
