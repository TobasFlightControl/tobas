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

  template <typename RepType = int64_t, typename DurType = std::ratio<1L>>
  bool sendGoalAndWait(
    const ActionType::Goal& goal,
    std::chrono::duration<RepType, DurType> timeout = std::chrono::seconds(0))
  {
    if (!client_->action_server_is_ready())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << action_name_ << "\" action server is not ready.");
      return false;
    }

    auto send_goal_future = client_->async_send_goal(goal);
    if (timeout.count() > 0)
    {
      if (send_goal_future.wait_for(timeout) == std::future_status::timeout)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Timeout before \"" << action_name_ << "\" response.");
        return false;
      }
    }
    else
    {
      send_goal_future.wait();
    }
    const auto goal_handle = send_goal_future.get();

    auto get_result_future = client_->async_get_result(goal_handle);
    if (timeout.count() > 0)
    {
      if (get_result_future.wait_for(timeout) == std::future_status::timeout)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Timeout before \"" << action_name_ << "\" response.");
        return false;
      }
    }
    else
    {
      get_result_future.wait();
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
