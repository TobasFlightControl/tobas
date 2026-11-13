// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/node.hpp>

#include "./definitions.hpp"
#include "./qos.hpp"

namespace tobas::ros2
{
template <typename MsgType>
PublisherPtr<MsgType> createPublisher(
  rclcpp::Node::SharedPtr _node,
  const std::string& _topic_name,
  bool _latch = qos::kDefaultLatch,
  bool _reliable = qos::kDefaultReliable,
  size_t _queue_size = qos::kDefaultQueueSize)
{
  return _node->create_publisher<MsgType>(_topic_name, qos::QoS(_latch, _reliable, _queue_size));
}

template <typename MsgType, typename Obj>
SubscriberPtr<MsgType> createSubscriber(
  rclcpp::Node::SharedPtr _node,
  const std::string& _topic_name,
  void (Obj::*_fp)(const std::shared_ptr<const MsgType>&),
  Obj* _obj,
  bool _latch = qos::kDefaultLatch,
  bool _reliable = qos::kDefaultReliable,
  size_t _queue_size = qos::kDefaultQueueSize)
{
  return _node->create_subscription<MsgType>(
    _topic_name, qos::QoS(_latch, _reliable, _queue_size), std::bind(_fp, _obj, std::placeholders::_1));
}

template <typename SrvType, typename Obj>
ServiceServerPtr<SrvType> createService(
  rclcpp::Node::SharedPtr _node,
  const std::string& _srv_name,
  void (Obj::*_fp)(
    const std::shared_ptr<const typename SrvType::Request>&,
    const std::shared_ptr<typename SrvType::Response>&),
  Obj* _obj)
{
  return _node->create_service<SrvType>(_srv_name, std::bind(_fp, _obj, std::placeholders::_1, std::placeholders::_2));
}

template <typename RepType, typename DurType, typename Obj>
ros2::TimerPtr createTimer(
  rclcpp::Node::SharedPtr _node,
  std::chrono::duration<RepType, DurType> _period,
  void (Obj::*_fp)(void),
  Obj* _obj)
{
  return _node->create_timer(_period, std::bind(_fp, _obj));
}
}  // namespace tobas::ros2
