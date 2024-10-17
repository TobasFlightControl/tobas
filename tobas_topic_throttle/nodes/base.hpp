#pragma once

#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/rate_manager.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

template <typename MsgType, const char* TopicName>
class TopicThrottleNode : public tobas::BaseNode
{
  static constexpr double kPublishRate = 25.;  // [Hz]

  using self = TopicThrottleNode;
  using super = tobas::BaseNode;

public:
  explicit TopicThrottleNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::RateManager rate_manager_;

  ros2::PublisherPtr<MsgType> pub_;
  ros2::SubscriberPtr<MsgType> sub_;

  void callback(const typename MsgType::ConstSharedPtr& msg_in);
};

template <typename MsgType, const char* TopicName>
TopicThrottleNode<MsgType, TopicName>::TopicThrottleNode(const rclcpp::NodeOptions& options)
  : super(std::string(TopicName) + "_throttle", options), rate_manager_(kPublishRate)
{
  static_assert(TopicName[0] != '/');
  pub_ = createPublisher<MsgType>(path::join(tobas::kThrottledTopicPrefix, TopicName));
  sub_ = createSubscriber(TopicName, &self::callback, this);
}

template <typename MsgType, const char* TopicName>
void TopicThrottleNode<MsgType, TopicName>::callback(const typename MsgType::ConstSharedPtr& msg_in)
{
  if (!rate_manager_.update(msg_in->header.stamp))
    return;

  auto msg_out = std::make_unique<MsgType>(*msg_in);
  pub_->publish(std::move(msg_out));
}
