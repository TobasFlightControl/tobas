#pragma once

#include <rclcpp/rclcpp.hpp>

namespace tobas_topic_throttle
{
template <typename MsgType, const char* TopicName>
class TopicThrottle
{
  static constexpr size_t kPublishRate = 30;  // [Hz]

  using self = TopicThrottle;

public:
  explicit TopicThrottle(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Rate rate_;
  typename MsgType::ConstSharedPtr msg_;

  PublisherPtr<> pub_;
  SubscriberPtr<> sub_;

  void callback(const typename MsgType::ConstSharedPtr& msg);
};

template <typename MsgType, const char* TopicName>
TopicThrottle<MsgType, TopicName>::TopicThrottle(rclcpp::Node::SharedPtr node) : rate_(kPublishRate)
{
  ROS_ASSERT(TopicName[0] != '/');
  pub_ = node.advertise<MsgType>(std::string("throttled/") + TopicName);
  sub_ = node.subscribe(TopicName, 1, &self::callback, this, rclcpp::TransportHints().tcpNoDelay());
}

template <typename MsgType, const char* TopicName>
void TopicThrottle<MsgType, TopicName>::callback(const typename MsgType::ConstSharedPtr& msg)
{
  if (msg_ == nullptr)
  {
    RCLCPP_INFO_STREAM("First \"" << TopicName << "\" is received.");
    rate_.reset();
  }

  msg_ = msg;
  pub_->publish(msg);
  rate_.sleep();
}
}  // namespace tobas_topic_throttle
