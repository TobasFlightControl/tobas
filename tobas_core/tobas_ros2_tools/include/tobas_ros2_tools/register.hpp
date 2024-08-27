#pragma once

#include "./definitions.hpp"
#include "./qos.hpp"

namespace ros2
{
template <typename MsgType>
PublisherPtr<MsgType> createPublisher(
  rclcpp::Node::SharedPtr node,
  const std::string& topic_name,
  bool latch = false,
  bool reliable = false,
  size_t queue_size = 1)
{
  return node->create_publisher<MsgType>(topic_name, makeQoS(latch, reliable, queue_size));
}

template <typename MsgType, typename Obj>
SubscriberPtr<MsgType> createSubscriber(
  rclcpp::Node::SharedPtr node,
  const std::string& topic_name,
  void (Obj::*fp)(const std::shared_ptr<const MsgType>&),
  Obj* obj,
  bool latch = false,
  bool reliable = false,
  size_t queue_size = 1)
{
  return node->create_subscription<MsgType>(
    topic_name, makeQoS(latch, reliable, queue_size), std::bind(fp, obj, std::placeholders::_1));
}

template <typename SrvType, typename Obj>
ServicePtr<SrvType> createService(
  rclcpp::Node::SharedPtr node,
  const std::string& srv_name,
  void (Obj::*fp)(
    const std::shared_ptr<const typename SrvType::Request>&,
    const std::shared_ptr<typename SrvType::Response>&),
  Obj* obj)
{
  return node->create_service<SrvType>(srv_name, std::bind(fp, obj, std::placeholders::_1, std::placeholders::_2));
}
}  // namespace ros2
