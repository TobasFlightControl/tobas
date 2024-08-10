#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>

#include <tobas_std_msgs/msg/message.hpp>

#define TOBAS_EXIT(...)                                                                                                \
  {                                                                                                                    \
    TOBAS_FATAL(__VA_ARGS__);                                                                                          \
    rclcpp::shutdown();                                                                                                \
  }

/* リリースモードでも機能するアサーション．ほとんど失敗し得ない操作の成否を一応確認するために使う． */
#define TOBAS_ASSERT(expr)                                                                                             \
  {                                                                                                                    \
    if (!static_cast<bool>(expr))                                                                                      \
    {                                                                                                                  \
      TOBAS_FATAL("Assertion failed: ", __FILE__, ": ", __LINE__);                                                     \
      rclcpp::shutdown();                                                                                              \
    }                                                                                                                  \
  }

#define TOBAS_DEBUG(...) debug(__VA_ARGS__)
#define TOBAS_INFO(...) info(__VA_ARGS__)
#define TOBAS_WARN(...) warn(__VA_ARGS__)
#define TOBAS_ERROR(...) error(__VA_ARGS__)
#define TOBAS_FATAL(...) fatal(__VA_ARGS__)

#define TOBAS_DEBUG_ONCE(...) debugOnce(__FILE__, __LINE__, __VA_ARGS__)
#define TOBAS_INFO_ONCE(...) infoOnce(__FILE__, __LINE__, __VA_ARGS__)
#define TOBAS_WARN_ONCE(...) warnOnce(__FILE__, __LINE__, __VA_ARGS__)
#define TOBAS_ERROR_ONCE(...) errorOnce(__FILE__, __LINE__, __VA_ARGS__)
#define TOBAS_FATAL_ONCE(...) fatalOnce(__FILE__, __LINE__, __VA_ARGS__)

#define TOBAS_DEBUG_THROTTLE(period, ...) debugThrottle(__FILE__, __LINE__, period, __VA_ARGS__)
#define TOBAS_INFO_THROTTLE(period, ...) infoThrottle(__FILE__, __LINE__, period, __VA_ARGS__)
#define TOBAS_WARN_THROTTLE(period, ...) warnThrottle(__FILE__, __LINE__, period, __VA_ARGS__)
#define TOBAS_ERROR_THROTTLE(period, ...) errorThrottle(__FILE__, __LINE__, period, __VA_ARGS__)
#define TOBAS_FATAL_THROTTLE(period, ...) fatalThrottle(__FILE__, __LINE__, period, __VA_ARGS__)

namespace gazebo
{
class BaseNode
{
public:
  explicit BaseNode(const std::string& name);

protected:
  template <typename MsgType>
  using PublisherPtr = rclcpp::Publisher<MsgType>::SharedPtr;
  template <typename MsgType>
  using SubscriberPtr = rclcpp::Subscription<MsgType>::SharedPtr;
  template <typename SrvType>
  using ServicePtr = rclcpp::Service<SrvType>::SharedPtr;
  template <typename SrvType>
  using ClientPtr = rclcpp::Client<SrvType>::SharedPtr;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  void initialize(const sdf::ElementConstPtr& sdf);

  const std::string& name() const;
  const std::string& ns() const;

  template <typename MsgType>
  PublisherPtr<MsgType>
  createPublisher(const std::string& topic_name, bool latch = false, bool reliable = true, size_t queue_size = 1);

  template <typename MsgType, typename Obj>
  SubscriberPtr<MsgType> createSubscriber(
    const std::string& topic_name,
    void (Obj::*fp)(const std::shared_ptr<const MsgType>&),
    Obj* obj,
    bool latch = false,
    bool reliable = true,
    size_t queue_size = 1);

  template <typename SrvType, typename Obj>
  ServicePtr<SrvType> createService(
    const std::string& srv_name,
    void (Obj::*fp)(
      const std::shared_ptr<const typename SrvType::Request>&,
      const std::shared_ptr<typename SrvType::Response>&),
    Obj* obj);

  template <typename... Args>
  void log(uint8_t level, const Args&... args) const;
  template <typename... Args>
  void logOnce(const char* file, int line, uint8_t level, const Args&... args);
  template <typename... Args>
  void logThrottle(const char* file, int line, uint8_t level, double period, const Args&... args);

  template <typename... Args>
  inline void debug(const Args&... args) const;
  template <typename... Args>
  inline void info(const Args&... args) const;
  template <typename... Args>
  inline void warn(const Args&... args) const;
  template <typename... Args>
  inline void error(const Args&... args) const;
  template <typename... Args>
  inline void fatal(const Args&... args) const;

  template <typename... Args>
  inline void debugOnce(const char* file, int line, const Args&... args);
  template <typename... Args>
  inline void infoOnce(const char* file, int line, const Args&... args);
  template <typename... Args>
  inline void warnOnce(const char* file, int line, const Args&... args);
  template <typename... Args>
  inline void errorOnce(const char* file, int line, const Args&... args);
  template <typename... Args>
  inline void fatalOnce(const char* file, int line, const Args&... args);

  template <typename... Args>
  inline void debugThrottle(const char* file, int line, double period, const Args&... args);
  template <typename... Args>
  inline void infoThrottle(const char* file, int line, double period, const Args&... args);
  template <typename... Args>
  inline void warnThrottle(const char* file, int line, double period, const Args&... args);
  template <typename... Args>
  inline void errorThrottle(const char* file, int line, double period, const Args&... args);
  template <typename... Args>
  inline void fatalThrottle(const char* file, int line, double period, const Args&... args);

private:
  const std::string name_;
  std::string ns_;

  std::unordered_set<std::string> log_once_;
  std::unordered_map<std::string, rclcpp::Time> log_throttle_;

  PublisherPtr<tobas_std_msgs::msg::Message> message_pub_;

  void rclcppLog(uint8_t level, const std::string& text) const;

  static rclcpp::QoS makeQoS(bool latch, bool reliable, size_t queue_size);
  static std::string createID(const char* file, int line);
};

template <typename MsgType>
BaseNode::PublisherPtr<MsgType>
BaseNode::createPublisher(const std::string& topic_name, bool latch, bool reliable, size_t queue_size)
{
  return node_->create_publisher<MsgType>(topic_name, makeQoS(latch, reliable, queue_size));
}

template <typename MsgType, typename Obj>
BaseNode::SubscriberPtr<MsgType> BaseNode::createSubscriber(
  const std::string& topic_name,
  void (Obj::*fp)(const std::shared_ptr<const MsgType>&),
  Obj* obj,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  return node_->create_subscription<MsgType>(
    topic_name, makeQoS(latch, reliable, queue_size), std::bind(fp, obj, std::placeholders::_1));
}

template <typename SrvType, typename Obj>
BaseNode::ServicePtr<SrvType> BaseNode::createService(
  const std::string& srv_name,
  void (Obj::*fp)(
    const std::shared_ptr<const typename SrvType::Request>&,
    const std::shared_ptr<typename SrvType::Response>&),
  Obj* obj)
{
  return node_->create_service<SrvType>(srv_name, std::bind(fp, obj, std::placeholders::_1, std::placeholders::_2));
}

template <typename... Args>
void BaseNode::log(uint8_t level, const Args&... args) const
{
  // Create message
  auto message = std::make_unique<tobas_std_msgs::msg::Message>();
  message->stamp = get_clock()->now();
  message->level = level;
  message->name = get_name();
  message->message = tobas_std::buildString(args...);

  // Output message to the console
  rclcppLog(level, message->message);

  // Publish message
  message_pub_->publish(std::move(message));
}

template <typename... Args>
void BaseNode::logOnce(const char* file, int line, uint8_t level, const Args&... args)
{
  const auto id = createID(file, line);
  if (log_once_.contains(id))
    return;
  log(level, args...);
  log_once_.insert(id);
}

template <typename... Args>
void BaseNode::logThrottle(const char* file, int line, uint8_t level, double period, const Args&... args)
{
  const auto id = createID(file, line);
  const auto now = get_clock()->now();
  auto it = log_throttle_.find(id);
  if (it == log_throttle_.end())
  {
    log(level, args...);
    log_throttle_[id] = now;
  }
  else
  {
    const auto diff = (now - it->second).seconds();
    if (diff > period)
    {
      log(level, args...);
      it->second = now;
    }
  }
}

template <typename... Args>
inline void BaseNode::debug(const Args&... args) const
{
  log(tobas_std_msgs::msg::Message::LEVEL_DEBUG, args...);
}

template <typename... Args>
inline void BaseNode::info(const Args&... args) const
{
  log(tobas_std_msgs::msg::Message::LEVEL_INFO, args...);
}

template <typename... Args>
inline void BaseNode::warn(const Args&... args) const
{
  log(tobas_std_msgs::msg::Message::LEVEL_WARN, args...);
}

template <typename... Args>
inline void BaseNode::error(const Args&... args) const
{
  log(tobas_std_msgs::msg::Message::LEVEL_ERROR, args...);
}

template <typename... Args>
inline void BaseNode::fatal(const Args&... args) const
{
  log(tobas_std_msgs::msg::Message::LEVEL_FATAL, args...);
}

template <typename... Args>
inline void BaseNode::debugOnce(const char* file, int line, const Args&... args)
{
  logOnce(file, line, tobas_std_msgs::msg::Message::LEVEL_DEBUG, args...);
}

template <typename... Args>
inline void BaseNode::infoOnce(const char* file, int line, const Args&... args)
{
  logOnce(file, line, tobas_std_msgs::msg::Message::LEVEL_INFO, args...);
}

template <typename... Args>
inline void BaseNode::warnOnce(const char* file, int line, const Args&... args)
{
  logOnce(file, line, tobas_std_msgs::msg::Message::LEVEL_WARN, args...);
}

template <typename... Args>
inline void BaseNode::errorOnce(const char* file, int line, const Args&... args)
{
  logOnce(file, line, tobas_std_msgs::msg::Message::LEVEL_ERROR, args...);
}

template <typename... Args>
inline void BaseNode::fatalOnce(const char* file, int line, const Args&... args)
{
  logOnce(file, line, tobas_std_msgs::msg::Message::LEVEL_FATAL, args...);
}

template <typename... Args>
inline void BaseNode::debugThrottle(const char* file, int line, double period, const Args&... args)
{
  logThrottle(file, line, tobas_std_msgs::msg::Message::LEVEL_DEBUG, period, args...);
}

template <typename... Args>
inline void BaseNode::infoThrottle(const char* file, int line, double period, const Args&... args)
{
  logThrottle(file, line, tobas_std_msgs::msg::Message::LEVEL_INFO, period, args...);
}

template <typename... Args>
inline void BaseNode::warnThrottle(const char* file, int line, double period, const Args&... args)
{
  logThrottle(file, line, tobas_std_msgs::msg::Message::LEVEL_WARN, period, args...);
}

template <typename... Args>
inline void BaseNode::errorThrottle(const char* file, int line, double period, const Args&... args)
{
  logThrottle(file, line, tobas_std_msgs::msg::Message::LEVEL_ERROR, period, args...);
}

template <typename... Args>
inline void BaseNode::fatalThrottle(const char* file, int line, double period, const Args&... args)
{
  logThrottle(file, line, tobas_std_msgs::msg::Message::LEVEL_FATAL, period, args...);
}
}  // namespace gazebo
