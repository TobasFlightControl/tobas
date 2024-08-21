#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace ros2
{
template <typename MsgType>
using PublisherPtr = rclcpp::Publisher<MsgType>::SharedPtr;
template <typename MsgType>
using SubscriberPtr = rclcpp::Subscription<MsgType>::SharedPtr;
template <typename SrvType>
using ServicePtr = rclcpp::Service<SrvType>::SharedPtr;
template <typename ActionType>
using ActionPtr = rclcpp_action::Server<ActionType>::SharedPtr;
template <typename ActionType>
using ActionGoalHandlePtr = std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>>;

using TimerPtr = rclcpp::TimerBase::SharedPtr;
using ParamHandlePtr = std::shared_ptr<rclcpp::ParameterCallbackHandle>;
}  // namespace ros2
