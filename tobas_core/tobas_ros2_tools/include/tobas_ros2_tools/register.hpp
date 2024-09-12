#pragma once

#include "./definitions.hpp"
#include "./qos.hpp"

namespace ros2
{
template <typename MsgType>
PublisherPtr<MsgType> createPublisher(
  rclcpp::Node::SharedPtr node,
  const std::string& topic_name,
  bool latch = qos::kDefaultLatch,
  bool reliable = qos::kDefaultReliable,
  size_t queue_size = qos::kDefaultQueueSize)
{
  return node->create_publisher<MsgType>(topic_name, makeQoS(latch, reliable, queue_size));
}

template <typename MsgType, typename Obj>
SubscriberPtr<MsgType> createSubscriber(
  rclcpp::Node::SharedPtr node,
  const std::string& topic_name,
  void (Obj::*fp)(const std::shared_ptr<const MsgType>&),
  Obj* obj,
  bool latch = qos::kDefaultLatch,
  bool reliable = qos::kDefaultReliable,
  size_t queue_size = qos::kDefaultQueueSize)
{
  return node->create_subscription<MsgType>(
    topic_name, makeQoS(latch, reliable, queue_size), std::bind(fp, obj, std::placeholders::_1));
}

template <typename SrvType, typename Obj>
ServiceServerPtr<SrvType> createService(
  rclcpp::Node::SharedPtr node,
  const std::string& srv_name,
  void (Obj::*fp)(
    const std::shared_ptr<const typename SrvType::Request>&,
    const std::shared_ptr<typename SrvType::Response>&),
  Obj* obj)
{
  return node->create_service<SrvType>(srv_name, std::bind(fp, obj, std::placeholders::_1, std::placeholders::_2));
}

template <typename RepType, typename DurType, typename Obj>
ros2::TimerPtr createTimer(
  rclcpp::Node::SharedPtr node,
  std::chrono::duration<RepType, DurType> period,
  void (Obj::*fp)(void),
  Obj* obj)
{
  return node->create_timer(period, std::bind(fp, obj));
}
}  // namespace ros2
