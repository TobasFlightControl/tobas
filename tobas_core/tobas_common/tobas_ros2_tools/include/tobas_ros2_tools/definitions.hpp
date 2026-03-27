#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace tobas
{
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
template <typename ActType>
using ActionServerPtr = typename rclcpp_action::Server<ActType>::SharedPtr;
template <typename ActType>
using ActionClientPtr = typename rclcpp_action::Client<ActType>::SharedPtr;

using TimerPtr = rclcpp::TimerBase::SharedPtr;
using ParamHandlePtr = std::shared_ptr<rclcpp::ParameterCallbackHandle>;
}  // namespace ros2
}  // namespace tobas
