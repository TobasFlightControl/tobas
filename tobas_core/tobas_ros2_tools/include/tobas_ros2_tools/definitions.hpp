#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace ros2
{
template <typename MsgType>
using PublisherPtr = typename rclcpp::Publisher<MsgType>::SharedPtr;
template <typename MsgType>
using SubscriberPtr = typename rclcpp::Subscription<MsgType>::SharedPtr;
template <typename SrvType>
using ServiceServerPtr = typename rclcpp::Service<SrvType>::SharedPtr;
template <typename SrvType>
using ServiceClientPtr = typename rclcpp::Client<SrvType>::SharedPtr;
template <typename ActionType>
using ActionServerPtr = typename rclcpp_action::Server<ActionType>::SharedPtr;
template <typename ActionType>
using ActionGoalHandlePtr = typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>>;

using TimerPtr = rclcpp::TimerBase::SharedPtr;
using ParamHandlePtr = std::shared_ptr<rclcpp::ParameterCallbackHandle>;
}  // namespace ros2
