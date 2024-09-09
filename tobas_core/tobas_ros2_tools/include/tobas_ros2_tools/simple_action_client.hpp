#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace ros2
{
template <typename ActionType>
class SimpleActionClient
{
public:
  using SharedPtr = std::shared_ptr<SimpleActionClient>;

  inline explicit SimpleActionClient(rclcpp::Node::SharedPtr node, const std::string& action_name)
    : node_(node), action_name_(action_name)
  {
    client_ = rclcpp_action::create_client<ActionType>(node, action_name);
  }

  bool sendGoalAndWait(
    const ActionType::Goal& goal,
    std::chrono::milliseconds get_result_timeout = std::chrono::milliseconds::max(),
    std::chrono::milliseconds send_goal_timeout = std::chrono::milliseconds(1000),
    std::chrono::milliseconds cancel_goal_timeout = std::chrono::milliseconds(1000))
  {
    if (!client_->action_server_is_ready())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << action_name_ << "\" action server is not ready.");
      return false;
    }

    auto send_goal_future = client_->async_send_goal(goal);
    if (send_goal_future.wait_for(send_goal_timeout) == std::future_status::timeout)
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
    if (get_result_future.wait_for(get_result_timeout) == std::future_status::timeout)
    {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(), "Timeout before getting \"" << action_name_ << "\" action result. Cancelling goal...");

      auto cancel_goal_future = client_->async_cancel_goal(goal_handle);
      if (cancel_goal_future.wait_for(cancel_goal_timeout) == std::future_status::timeout)
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
