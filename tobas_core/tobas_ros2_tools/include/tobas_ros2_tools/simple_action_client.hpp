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

  bool sendGoalAndWait(const ActionType::Goal& goal)
  {
    if (!client_->action_server_is_ready())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << action_name_ << "\" action server is not ready.");
      return false;
    }

    auto send_goal_id = client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node_->shared_from_this(), send_goal_id) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to send \"" << action_name_ << "\" action goal.");
      return false;
    }
    const auto goal_handle = send_goal_id.get();

    auto get_result_id = client_->async_get_result(goal_handle);
    if (
      rclcpp::spin_until_future_complete(node_->shared_from_this(), get_result_id) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get \"" << action_name_ << "\" action result.");
      return false;
    }
    result_ = get_result_id.get();

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
