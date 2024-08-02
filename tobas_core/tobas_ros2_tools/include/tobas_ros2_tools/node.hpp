#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tobas_std_tools/debug.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/stream.hpp>
#include <tobas_std_tools/unordered_set.hpp>
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

namespace ros2
{
class Node : public rclcpp::Node
{
  static constexpr char kMessageTopic[] = "message";

  using super = rclcpp::Node;
  using self = Node;

public:
  explicit Node(const std::string& node_name, const rclcpp::NodeOptions& options);

protected:
  template <typename MsgType>
  using PublisherPtr = rclcpp::Publisher<MsgType>::SharedPtr;
  template <typename MsgType>
  using SubscriberPtr = rclcpp::Subscription<MsgType>::SharedPtr;
  template <typename SrvType>
  using ServicePtr = rclcpp::Service<SrvType>::SharedPtr;
  template <typename SrvType>
  using ClientPtr = rclcpp::Client<SrvType>::SharedPtr;

  using TimerPtr = rclcpp::TimerBase::SharedPtr;

  inline std::string ns() const;

  template <typename MsgType>
  PublisherPtr<MsgType> createPublisher(const std::string& topic_name, const rclcpp::QoS& qos);

  template <typename MsgType, typename Obj>
  SubscriberPtr<MsgType> createSubscriber(
    const std::string& topic_name,
    const rclcpp::QoS& qos,
    void (Obj::*fp)(const typename MsgType::ConstSharedPtr&),
    Obj* obj);

  template <typename SrvType, typename Obj>
  typename rclcpp::Service<SrvType>::SharedPtr createService(
    const std::string& srv_name,
    void (Obj::*fp)(const typename SrvType::Request::ConstSharedPtr&, const typename SrvType::Response::SharedPtr&),
    Obj* obj);

  template <typename DurationRepType, typename DurationType, typename Obj>
  TimerPtr createTimer(std::chrono::duration<DurationRepType, DurationType> period, void (Obj::*fp)(void), Obj* obj);

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

  bool getBoolParam(const std::string& name);
  long getIntParam(const std::string& name);
  double getDoubleParam(const std::string& name);
  std::string getStringParam(const std::string& name);
  std::vector<bool> getBoolArrayParam(const std::string& name);
  std::vector<uint8_t> getByteArrayParam(const std::string& name);
  std::vector<long> getIntArrayParam(const std::string& name);
  std::vector<double> getDoubleArrayParam(const std::string& name);
  std::vector<std::string> getStringArrayParam(const std::string& name);

  bool getBoolParam(const std::string& name, const bool& _default);
  long getIntParam(const std::string& name, const long& _default);
  double getDoubleParam(const std::string& name, const double& _default);
  std::string getStringParam(const std::string& name, const std::string& _default);
  std::vector<bool> getBoolArrayParam(const std::string& name, const std::vector<bool>& _default);
  std::vector<uint8_t> getByteArrayParam(const std::string& name, const std::vector<uint8_t>& _default);
  std::vector<long> getIntArrayParam(const std::string& name, const std::vector<long>& _default);
  std::vector<double> getDoubleArrayParam(const std::string& name, const std::vector<double>& _default);
  std::vector<std::string> getStringArrayParam(const std::string& name, const std::vector<std::string>& _default);

private:
  std::unordered_set<std::string> log_once_;
  std::unordered_map<std::string, rclcpp::Time> log_throttle_;

  rclcpp::Publisher<tobas_std_msgs::msg::Message>::SharedPtr message_pub_;

  template <typename T>
  void declareParam(const std::string& name, const T& _default);

  void rclcppLog(uint8_t level, const std::string& text) const;

  inline static std::string createID(const char* file, int line);
};

inline std::string Node::ns() const
{
  return std::string(get_namespace()) + "/";
}

template <typename MsgType>
Node::PublisherPtr<MsgType> Node::createPublisher(const std::string& topic_name, const rclcpp::QoS& qos)
{
  return create_publisher<MsgType>(topic_name, qos);
}

template <typename MsgType, typename Obj>
Node::SubscriberPtr<MsgType> Node::createSubscriber(
  const std::string& topic_name,
  const rclcpp::QoS& qos,
  void (Obj::*fp)(const typename MsgType::ConstSharedPtr&),
  Obj* obj)
{
  return create_subscription<MsgType>(topic_name, qos, std::bind(fp, obj, std::placeholders::_1));
}

template <typename SrvType, typename Obj>
typename rclcpp::Service<SrvType>::SharedPtr Node::createService(
  const std::string& srv_name,
  void (Obj::*fp)(const typename SrvType::Request::ConstSharedPtr&, const typename SrvType::Response::SharedPtr&),
  Obj* obj)
{
  return create_service<SrvType>(srv_name, std::bind(fp, obj, std::placeholders::_1, std::placeholders::_2));
}

template <typename DurationRepT, typename DurationT, typename Obj>
Node::TimerPtr Node::createTimer(std::chrono::duration<DurationRepT, DurationT> period, void (Obj::*fp)(void), Obj* obj)
{
  return create_timer(period, bind(fp, obj));
}

template <typename... Args>
void Node::log(uint8_t level, const Args&... args) const
{
  // Create message
  auto message = std::make_unique<tobas_std_msgs::msg::Message>();
  message->header.stamp = get_clock()->now();
  message->level = level;
  message->name = get_name();
  message->message = tobas_std::buildString(args...);

  // Output message to the console
  rclcppLog(level, "[" + message->name + "] " + message->message);

  // Publish message
  message_pub_->publish(std::move(message));
}

template <typename... Args>
void Node::logOnce(const char* file, int line, uint8_t level, const Args&... args)
{
  const auto id = createID(file, line);
  if (tobas_std::contains(log_once_, id))
    return;
  log(level, args...);
  log_once_.insert(id);
}

template <typename... Args>
void Node::logThrottle(const char* file, int line, uint8_t level, double period, const Args&... args)
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
inline void Node::debug(const Args&... args) const
{
  log(tobas_std_msgs::msg::Message::LEVEL_DEBUG, args...);
}

template <typename... Args>
inline void Node::info(const Args&... args) const
{
  log(tobas_std_msgs::msg::Message::LEVEL_INFO, args...);
}

template <typename... Args>
inline void Node::warn(const Args&... args) const
{
  log(tobas_std_msgs::msg::Message::LEVEL_WARN, args...);
}

template <typename... Args>
inline void Node::error(const Args&... args) const
{
  log(tobas_std_msgs::msg::Message::LEVEL_ERROR, args...);
}

template <typename... Args>
inline void Node::fatal(const Args&... args) const
{
  log(tobas_std_msgs::msg::Message::LEVEL_FATAL, args...);
}

template <typename... Args>
inline void Node::debugOnce(const char* file, int line, const Args&... args)
{
  logOnce(file, line, tobas_std_msgs::msg::Message::LEVEL_DEBUG, args...);
}

template <typename... Args>
inline void Node::infoOnce(const char* file, int line, const Args&... args)
{
  logOnce(file, line, tobas_std_msgs::msg::Message::LEVEL_INFO, args...);
}

template <typename... Args>
inline void Node::warnOnce(const char* file, int line, const Args&... args)
{
  logOnce(file, line, tobas_std_msgs::msg::Message::LEVEL_WARN, args...);
}

template <typename... Args>
inline void Node::errorOnce(const char* file, int line, const Args&... args)
{
  logOnce(file, line, tobas_std_msgs::msg::Message::LEVEL_ERROR, args...);
}

template <typename... Args>
inline void Node::fatalOnce(const char* file, int line, const Args&... args)
{
  logOnce(file, line, tobas_std_msgs::msg::Message::LEVEL_FATAL, args...);
}

template <typename... Args>
inline void Node::debugThrottle(const char* file, int line, double period, const Args&... args)
{
  logThrottle(file, line, tobas_std_msgs::msg::Message::LEVEL_DEBUG, period, args...);
}

template <typename... Args>
inline void Node::infoThrottle(const char* file, int line, double period, const Args&... args)
{
  logThrottle(file, line, tobas_std_msgs::msg::Message::LEVEL_INFO, period, args...);
}

template <typename... Args>
inline void Node::warnThrottle(const char* file, int line, double period, const Args&... args)
{
  logThrottle(file, line, tobas_std_msgs::msg::Message::LEVEL_WARN, period, args...);
}

template <typename... Args>
inline void Node::errorThrottle(const char* file, int line, double period, const Args&... args)
{
  logThrottle(file, line, tobas_std_msgs::msg::Message::LEVEL_ERROR, period, args...);
}

template <typename... Args>
inline void Node::fatalThrottle(const char* file, int line, double period, const Args&... args)
{
  logThrottle(file, line, tobas_std_msgs::msg::Message::LEVEL_FATAL, period, args...);
}

template <typename T>
void Node::declareParam(const std::string& name, const T& _default)
{
  try
  {
    declare_parameter<T>(name);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
    RCLCPP_WARN_STREAM(get_logger(), "Parameter \"" << name << "\" is already declared.");
    return;
  }
  catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException&)
  {
    RCLCPP_WARN_STREAM(
      get_logger(), "Parameter \"" << name << "\" is not specified. The default \"" << _default << "\" is set.");
    declare_parameter(name, _default);
  }
}

inline std::string Node::createID(const char* file, int line)
{
  return std::string(file) + ":" + std::to_string(line);
}
}  // namespace ros2
