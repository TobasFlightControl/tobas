#pragma once

#include <tobas_path_tools/join.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

template <typename MsgType, const char* TopicName>
class TopicThrottleNode : public tobas::BaseNode
{
  static constexpr size_t kPublishRate = 30;  // [Hz]

  using self = TopicThrottleNode;
  using super = tobas::BaseNode;

public:
  explicit TopicThrottleNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::Rate rate_;
  typename MsgType::ConstSharedPtr msg_in_;

  ros2::PublisherPtr<MsgType> pub_;
  ros2::SubscriberPtr<MsgType> sub_;

  void callback(const typename MsgType::ConstSharedPtr& msg_in);
};

template <typename MsgType, const char* TopicName>
TopicThrottleNode<MsgType, TopicName>::TopicThrottleNode(const rclcpp::NodeOptions& options)
  : super(std::string(TopicName) + "_throttle", options), rate_(kPublishRate)
{
  static_assert(TopicName[0] != '/');
  pub_ = createPublisher<MsgType>(path::join(tobas::kThrottledTopicPrefix, TopicName));
  sub_ = createSubscriber(TopicName, &self::callback, this);
}

template <typename MsgType, const char* TopicName>
void TopicThrottleNode<MsgType, TopicName>::callback(const typename MsgType::ConstSharedPtr& msg_in)
{
  if (msg_in_ == nullptr)
  {
    TOBAS_INFO("First \"", TopicName, "\" is received.");
    rate_.reset();
  }

  auto msg_out = std::make_unique<MsgType>(*msg_in);
  pub_->publish(std::move(msg_out));

  msg_in_ = msg_in;
  rate_.sleep();
}
