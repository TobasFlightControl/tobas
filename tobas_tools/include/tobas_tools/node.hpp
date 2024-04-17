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

  explicit BaseNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const std::string& name);

  virtual void getRosParams()
  {
  }
  virtual void registerPublishers()
  {
  }
  virtual void registerSubscribers()
  {
  }

  inline const std::string& name() const;
  inline std::string ns() const;

  template <typename... Args>
  inline void debug(const Args&... args);
  template <typename... Args>
  inline void info(const Args&... args);
  template <typename... Args>
  inline void warn(const Args&... args);
  template <typename... Args>
  inline void error(const Args&... args);
  template <typename... Args>
  inline void fatal(const Args&... args);

  template <typename... Args>
  void debugThrottle(const double& period, const Args&... args);
  template <typename... Args>
  void infoThrottle(const double& period, const Args&... args);
  template <typename... Args>
  void warnThrottle(const double& period, const Args&... args);
  template <typename... Args>
  void errorThrottle(const double& period, const Args&... args);
  template <typename... Args>
  void fatalThrottle(const double& period, const Args&... args);

  template <typename... Args>
  void exit(const Args&... args);

  /* Alias for ros::TransportHints().reliable().tcpNoDelay(). */
  static ros::TransportHints tcpNoDelay(const bool& nodelay = true);

private:
  const std::string name_;

  std::map<std::string, ros::Time> log_throttle_;
  std::mutex log_throttle_mutex_;

  ros::Publisher message_pub_;

  template <typename... Args>
  void log(const uint8_t& level, const Args&... args);

  template <typename... Args>
  void logThrottle(const uint8_t& level, const double& period, const Args&... args);
};

inline const std::string& BaseNode::name() const
{
  return name_;
}

inline std::string BaseNode::ns() const
{
  return nh_.getNamespace() + "/";
}

template <typename... Args>
inline void BaseNode::debug(const Args&... args)
{
  log(tobas_msgs::Message::DEBUG, args...);
}

template <typename... Args>
inline void BaseNode::info(const Args&... args)
{
  log(tobas_msgs::Message::INFO, args...);
}

template <typename... Args>
inline void BaseNode::warn(const Args&... args)
{
  log(tobas_msgs::Message::WARN, args...);
}

template <typename... Args>
inline void BaseNode::error(const Args&... args)
{
  log(tobas_msgs::Message::ERROR, args...);
}

template <typename... Args>
inline void BaseNode::fatal(const Args&... args)
{
  log(tobas_msgs::Message::FATAL, args...);
}

template <typename... Args>
void BaseNode::debugThrottle(const double& period, const Args&... args)
{
  logThrottle(tobas_msgs::Message::DEBUG, period, args...);
}

template <typename... Args>
void BaseNode::infoThrottle(const double& period, const Args&... args)
{
  logThrottle(tobas_msgs::Message::INFO, period, args...);
}

template <typename... Args>
void BaseNode::warnThrottle(const double& period, const Args&... args)
{
  logThrottle(tobas_msgs::Message::WARN, period, args...);
}

template <typename... Args>
void BaseNode::errorThrottle(const double& period, const Args&... args)
{
  logThrottle(tobas_msgs::Message::ERROR, period, args...);
}

template <typename... Args>
void BaseNode::fatalThrottle(const double& period, const Args&... args)
{
  logThrottle(tobas_msgs::Message::FATAL, period, args...);
}

template <typename... Args>
void BaseNode::exit(const Args&... args)
{
  fatal(args...);
  nh_.shutdown();
}

template <typename... Args>
void BaseNode::log(const uint8_t& level, const Args&... args)
{
  const auto message = boost::make_shared<tobas_msgs::Message>();
  message->header.stamp = ros::Time::now();
  message->level = level;
  message->name = name_;
  message->message = tobas_std::buildString(args...);
  message_pub_.publish(message);
}

template <typename... Args>
void BaseNode::logThrottle(const uint8_t& level, const double& period, const Args&... args)
{
  const auto id = std::string(__FILE__) + ":" + std::to_string(__LINE__);
  const auto now = ros::Time::now();
  std::lock_guard<std::mutex> lock(log_throttle_mutex_);
  auto it = log_throttle_.find(id);
  if (it == log_throttle_.end())
  {
    log_throttle_[id] = now;
    log(level, args...);
  }
  else
  {
    const auto diff = (now - it->second).toSec();
    if (diff > period)
    {
      it->second = now;
      log(level, args...);
    }
  }
}
}  // namespace tobas
