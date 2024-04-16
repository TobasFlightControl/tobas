#pragma once

#include <ros/ros.h>

#include <tobas_std_tools/stream.hpp>
#include <tobas_msgs/Message.h>

namespace tobas
{
/**
 * @brief あらゆるノードの規定ノード．プログラミングの自由度を下げて見通しを良くすることが目的．
 */
class BaseNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  const std::string name_;

  explicit BaseNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const std::string& name);

  virtual void getRosParams() = 0;
  virtual void registerPublishers() = 0;
  virtual void registerSubscribers() = 0;

  inline std::string ns();

  template <typename... Args>
  void debug(Args... args);
  template <typename... Args>
  void info(Args... args);
  template <typename... Args>
  void warn(Args... args);
  template <typename... Args>
  void error(Args... args);
  template <typename... Args>
  void fatal(Args... args);

  /* Alias for ros::TransportHints().reliable().tcpNoDelay(). */
  static ros::TransportHints tcpNoDelay(const bool& nodelay = true);

private:
  ros::Publisher message_pub_;

  template <typename... Args>
  inline const tobas_msgs::MessagePtr createMessageCommon(const Args&... args);
};

inline std::string BaseNode::ns()
{
  return nh_.getNamespace() + "/";
}

template <typename... Args>
void BaseNode::debug(Args... args)
{
  const auto message = createMessageCommon(args...);
  message->level = tobas_msgs::Message::DEBUG;
  message_pub_.publish(message);
}

template <typename... Args>
void BaseNode::info(Args... args)
{
  const auto message = createMessageCommon(args...);
  message->level = tobas_msgs::Message::INFO;
  message_pub_.publish(message);
}

template <typename... Args>
void BaseNode::warn(Args... args)
{
  const auto message = createMessageCommon(args...);
  message->level = tobas_msgs::Message::WARN;
  message_pub_.publish(message);
}

template <typename... Args>
void BaseNode::error(Args... args)
{
  const auto message = createMessageCommon(args...);
  message->level = tobas_msgs::Message::ERROR;
  message_pub_.publish(message);
}

template <typename... Args>
void BaseNode::fatal(Args... args)
{
  const auto message = createMessageCommon(args...);
  message->level = tobas_msgs::Message::FATAL;
  message_pub_.publish(message);
}

template <typename... Args>
inline const tobas_msgs::MessagePtr BaseNode::createMessageCommon(const Args&... args)
{
  const auto message = boost::make_shared<tobas_msgs::Message>();
  message->header.stamp = ros::Time::now();
  message->name = name_;
  message->message = tobas_std::buildString(args...);
  return message;
}
}  // namespace tobas
