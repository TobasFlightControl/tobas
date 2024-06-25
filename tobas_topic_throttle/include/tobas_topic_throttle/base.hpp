#pragma once

#include <ros/ros.h>

namespace tobas_topic_throttle
{
template <typename MsgType, const char* TopicName>
class TopicThrottle
{
  static constexpr size_t kPublishRate = 30;  // [Hz]

  using self = TopicThrottle;

public:
  explicit TopicThrottle(ros::NodeHandle& nh);

private:
  ros::Rate rate_;
  MsgType::ConstPtr msg_;

  ros::Publisher pub_;
  ros::Subscriber sub_;

  void callback(const MsgType::ConstPtr& msg);
};

template <typename MsgType, const char* TopicName>
TopicThrottle<MsgType, TopicName>::TopicThrottle(ros::NodeHandle& nh) : rate_(kPublishRate)
{
  ROS_ASSERT(TopicName[0] != '/');
  pub_ = nh.advertise<MsgType>(std::string("throttled/") + TopicName, 1);
  sub_ = nh.subscribe(TopicName, 1, &self::callback, this, ros::TransportHints().tcpNoDelay());
}

template <typename MsgType, const char* TopicName>
void TopicThrottle<MsgType, TopicName>::callback(const typename MsgType::ConstPtr& msg)
{
  if (msg_ == nullptr)
  {
    ROS_INFO_STREAM("First \"" << TopicName << "\" is received.");
    rate_.reset();
  }

  msg_ = msg;
  pub_.publish(msg);
  rate_.sleep();
}
}  // namespace tobas_topic_throttle
