// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <concepts>
#include <optional>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <sdf/sdf.hh>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_std_tools/stream.hpp>

#include <tobas_msgs/msg/message.hpp>

#define TOBAS_EXIT(...)                                                                                                \
  {                                                                                                                    \
    TOBAS_FATAL(__VA_ARGS__);                                                                                          \
    throw;                                                                                                             \
  }

#define TOBAS_DEBUG(...) this->debug(__VA_ARGS__)
#define TOBAS_INFO(...) this->info(__VA_ARGS__)
#define TOBAS_WARN(...) this->warn(__VA_ARGS__)
#define TOBAS_ERROR(...) this->error(__VA_ARGS__)
#define TOBAS_FATAL(...) this->fatal(__VA_ARGS__)

#define TOBAS_DEBUG_ONCE(...) this->debugOnce(__FILE__, __LINE__, __VA_ARGS__)
#define TOBAS_INFO_ONCE(...) this->infoOnce(__FILE__, __LINE__, __VA_ARGS__)
#define TOBAS_WARN_ONCE(...) this->warnOnce(__FILE__, __LINE__, __VA_ARGS__)
#define TOBAS_ERROR_ONCE(...) this->errorOnce(__FILE__, __LINE__, __VA_ARGS__)
#define TOBAS_FATAL_ONCE(...) this->fatalOnce(__FILE__, __LINE__, __VA_ARGS__)

#define TOBAS_DEBUG_THROTTLE(period, ...) this->debugThrottle(__FILE__, __LINE__, period, __VA_ARGS__)
#define TOBAS_INFO_THROTTLE(period, ...) this->infoThrottle(__FILE__, __LINE__, period, __VA_ARGS__)
#define TOBAS_WARN_THROTTLE(period, ...) this->warnThrottle(__FILE__, __LINE__, period, __VA_ARGS__)
#define TOBAS_ERROR_THROTTLE(period, ...) this->errorThrottle(__FILE__, __LINE__, period, __VA_ARGS__)
#define TOBAS_FATAL_THROTTLE(period, ...) this->fatalThrottle(__FILE__, __LINE__, period, __VA_ARGS__)

namespace tobas
{
namespace gazebo
{
class BaseNode
{
public:
  explicit BaseNode();
  ~BaseNode();

protected:
  /** Constraints on SDF parameters. */
  enum SdfConstraint
  {
    kNone,
    kPositive,
    kNegative,
    kNonNegative,
    kNonPositive,
  };

  rclcpp::Node::SharedPtr node_;

  void initialize(const std::string& _name, const sdf::ElementConstPtr& _sdf);

  const std::string& name() const;
  const std::string& ns() const;

  template <typename MsgType>
  ros2::PublisherPtr<MsgType> createPublisher(
    const std::string& _topic_name,
    bool _latch = ros2::qos::kDefaultLatch,
    bool _reliable = ros2::qos::kDefaultReliable,
    size_t _queue_size = ros2::qos::kDefaultQueueSize);

  template <typename MsgType, typename Obj>
  ros2::SubscriberPtr<MsgType> createSubscriber(
    const std::string& _topic_name,
    void (Obj::*_fp)(const std::shared_ptr<const MsgType>&),
    Obj* _obj,
    bool _latch = ros2::qos::kDefaultLatch,
    bool _reliable = ros2::qos::kDefaultReliable,
    size_t _queue_size = ros2::qos::kDefaultQueueSize);

  template <typename SrvType, typename Obj>
  ros2::ServiceServerPtr<SrvType> createService(
    const std::string& _srv_name,
    void (Obj::*_fp)(
      const std::shared_ptr<const typename SrvType::Request>&,
      const std::shared_ptr<typename SrvType::Response>&),
    Obj* _obj);

  template <typename... Args>
  void log(uint8_t _level, const Args&... _args) const;
  template <typename... Args>
  void logOnce(const char* _file, int _line, uint8_t _level, const Args&... _args);
  template <typename... Args>
  void logThrottle(const char* _file, int _line, uint8_t _level, double _period, const Args&... _args);

  template <typename... Args>
  inline void debug(const Args&... _args) const;
  template <typename... Args>
  inline void info(const Args&... _args) const;
  template <typename... Args>
  inline void warn(const Args&... _args) const;
  template <typename... Args>
  inline void error(const Args&... _args) const;
  template <typename... Args>
  inline void fatal(const Args&... _args) const;

  template <typename... Args>
  inline void debugOnce(const char* _file, int _line, const Args&... _args);
  template <typename... Args>
  inline void infoOnce(const char* _file, int _line, const Args&... _args);
  template <typename... Args>
  inline void warnOnce(const char* _file, int _line, const Args&... _args);
  template <typename... Args>
  inline void errorOnce(const char* _file, int _line, const Args&... _args);
  template <typename... Args>
  inline void fatalOnce(const char* _file, int _line, const Args&... _args);

  template <typename... Args>
  inline void debugThrottle(const char* _file, int _line, double _period, const Args&... _args);
  template <typename... Args>
  inline void infoThrottle(const char* _file, int _line, double _period, const Args&... _args);
  template <typename... Args>
  inline void warnThrottle(const char* _file, int _line, double _period, const Args&... _args);
  template <typename... Args>
  inline void errorThrottle(const char* _file, int _line, double _period, const Args&... _args);
  template <typename... Args>
  inline void fatalThrottle(const char* _file, int _line, double _period, const Args&... _args);

  template <typename T>
  void checkConstraint(const std::string& _name, const T& _param, SdfConstraint _constr) const;

  template <typename T>
  T getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name) const;
  template <typename T>
  T getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name, const T& _dflt) const;
  template <typename T>
  T getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name, SdfConstraint _constr) const;
  template <typename T>
  T getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name, const T& _dflt, SdfConstraint _constr) const;
  template <typename T>
    requires std::same_as<T, std::vector<typename T::value_type, typename T::allocator_type>>
  T getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name) const;
  template <typename T>
    requires std::same_as<T, std::pair<typename T::first_type, typename T::first_type>>
  T getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name) const;

private:
  std::string name_;
  std::string ns_;

  std::unordered_set<std::string> log_once_;
  std::unordered_map<std::string, rclcpp::Time> log_throttle_;

  ros2::PublisherPtr<tobas_msgs::msg::Message> message_pub_;

  std::optional<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  void gazeboLog(uint8_t _level, const std::string& _text) const;

  static std::string createID(const char* _file, int _line);
};

template <typename MsgType>
ros2::PublisherPtr<MsgType>
BaseNode::createPublisher(const std::string& _topic_name, bool _latch, bool _reliable, size_t _queue_size)
{
  return ros2::createPublisher<MsgType>(node_, _topic_name, _latch, _reliable, _queue_size);
}

template <typename MsgType, typename Obj>
ros2::SubscriberPtr<MsgType> BaseNode::createSubscriber(
  const std::string& _topic_name,
  void (Obj::*_fp)(const std::shared_ptr<const MsgType>&),
  Obj* _obj,
  bool _latch,
  bool _reliable,
  size_t _queue_size)
{
  return ros2::createSubscriber<MsgType>(node_, _topic_name, _fp, _obj, _latch, _reliable, _queue_size);
}

template <typename SrvType, typename Obj>
ros2::ServiceServerPtr<SrvType> BaseNode::createService(
  const std::string& _srv_name,
  void (Obj::*_fp)(
    const std::shared_ptr<const typename SrvType::Request>&,
    const std::shared_ptr<typename SrvType::Response>&),
  Obj* _obj)
{
  return ros2::createService<SrvType>(node_, _srv_name, _fp, _obj);
}

template <typename... Args>
void BaseNode::log(uint8_t _level, const Args&... _args) const
{
  // Create message.
  auto message = std::make_unique<tobas_msgs::msg::Message>();
  message->header.stamp = node_->now();
  message->level = _level;
  message->name = node_->get_name();
  message->message = st::buildString(_args...);

  // Output message to the console.
  gazeboLog(_level, message->message);

  // Publish message.
  message_pub_->publish(std::move(message));
}

template <typename... Args>
void BaseNode::logOnce(const char* _file, int _line, uint8_t _level, const Args&... _args)
{
  const auto id = createID(_file, _line);
  if (log_once_.contains(id)) {
    return;
  }
  log(_level, _args...);
  log_once_.insert(id);
}

template <typename... Args>
void BaseNode::logThrottle(const char* _file, int _line, uint8_t _level, double _period, const Args&... _args)
{
  const auto id = createID(_file, _line);
  const auto cur_time = node_->now();
  auto it = log_throttle_.find(id);
  if (it == log_throttle_.end()) {
    log(_level, _args...);
    log_throttle_[id] = cur_time;
  }
  else {
    const auto diff = (cur_time - it->second).seconds();
    if (diff > _period) {
      log(_level, _args...);
      it->second = cur_time;
    }
  }
}

template <typename... Args>
inline void BaseNode::debug(const Args&... _args) const
{
  log(tobas_msgs::msg::Message::LEVEL_DEBUG, _args...);
}

template <typename... Args>
inline void BaseNode::info(const Args&... _args) const
{
  log(tobas_msgs::msg::Message::LEVEL_INFO, _args...);
}

template <typename... Args>
inline void BaseNode::warn(const Args&... _args) const
{
  log(tobas_msgs::msg::Message::LEVEL_WARN, _args...);
}

template <typename... Args>
inline void BaseNode::error(const Args&... _args) const
{
  log(tobas_msgs::msg::Message::LEVEL_ERROR, _args...);
}

template <typename... Args>
inline void BaseNode::fatal(const Args&... _args) const
{
  log(tobas_msgs::msg::Message::LEVEL_FATAL, _args...);
}

template <typename... Args>
inline void BaseNode::debugOnce(const char* _file, int _line, const Args&... _args)
{
  logOnce(_file, _line, tobas_msgs::msg::Message::LEVEL_DEBUG, _args...);
}

template <typename... Args>
inline void BaseNode::infoOnce(const char* _file, int _line, const Args&... _args)
{
  logOnce(_file, _line, tobas_msgs::msg::Message::LEVEL_INFO, _args...);
}

template <typename... Args>
inline void BaseNode::warnOnce(const char* _file, int _line, const Args&... _args)
{
  logOnce(_file, _line, tobas_msgs::msg::Message::LEVEL_WARN, _args...);
}

template <typename... Args>
inline void BaseNode::errorOnce(const char* _file, int _line, const Args&... _args)
{
  logOnce(_file, _line, tobas_msgs::msg::Message::LEVEL_ERROR, _args...);
}

template <typename... Args>
inline void BaseNode::fatalOnce(const char* _file, int _line, const Args&... _args)
{
  logOnce(_file, _line, tobas_msgs::msg::Message::LEVEL_FATAL, _args...);
}

template <typename... Args>
inline void BaseNode::debugThrottle(const char* _file, int _line, double _period, const Args&... _args)
{
  logThrottle(_file, _line, tobas_msgs::msg::Message::LEVEL_DEBUG, _period, _args...);
}

template <typename... Args>
inline void BaseNode::infoThrottle(const char* _file, int _line, double _period, const Args&... _args)
{
  logThrottle(_file, _line, tobas_msgs::msg::Message::LEVEL_INFO, _period, _args...);
}

template <typename... Args>
inline void BaseNode::warnThrottle(const char* _file, int _line, double _period, const Args&... _args)
{
  logThrottle(_file, _line, tobas_msgs::msg::Message::LEVEL_WARN, _period, _args...);
}

template <typename... Args>
inline void BaseNode::errorThrottle(const char* _file, int _line, double _period, const Args&... _args)
{
  logThrottle(_file, _line, tobas_msgs::msg::Message::LEVEL_ERROR, _period, _args...);
}

template <typename... Args>
inline void BaseNode::fatalThrottle(const char* _file, int _line, double _period, const Args&... _args)
{
  logThrottle(_file, _line, tobas_msgs::msg::Message::LEVEL_FATAL, _period, _args...);
}

template <typename T>
void BaseNode::checkConstraint(const std::string& _name, const T& _param, SdfConstraint _constr) const
{
  switch (_constr) {
    case kNone:
      break;
    case kPositive:
      if (_param <= 0) {
        TOBAS_EXIT(_name, " must be positive.");
      }
      break;
    case kNegative:
      if (_param >= 0) {
        TOBAS_EXIT(_name, " must be negative.");
      }
      break;
    case kNonNegative:
      if (_param < 0) {
        TOBAS_EXIT(_name, " must be non-negative.");
      }
      break;
    case kNonPositive:
      if (_param > 0) {
        TOBAS_EXIT(_name, " must be non-positive.");
      }
      break;
    default:
      TOBAS_EXIT("Invalid constr type.");
  }
}

template <typename T>
T BaseNode::getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name) const
{
  if (!_sdf->HasElement(_name)) {
    TOBAS_EXIT("Please specify '", _name, "'.");
  }
  return _sdf->Get<T>(_name);
}

template <typename T>
T BaseNode::getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name, const T& _dflt) const
{
  T param;
  if (!_sdf->Get(_name, param, _dflt)) {
    TOBAS_WARN("SDF parameter '", _name, "' is not specified. The default value '", _dflt, "' is used.");
  }
  return param;
}

template <typename T>
T BaseNode::getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name, SdfConstraint _constr) const
{
  const auto param = getSdfParam<T>(_sdf, _name);
  checkConstraint(_name, param, _constr);
  return param;
}

template <typename T>
T BaseNode::getSdfParam(
  const sdf::ElementConstPtr& _sdf,
  const std::string& _name,
  const T& _dflt,
  SdfConstraint _constr) const
{
  const auto param = getSdfParam<T>(_sdf, _name, _dflt);
  checkConstraint(_name, param, _constr);
  return param;
}

template <typename T>
  requires std::same_as<T, std::vector<typename T::value_type, typename T::allocator_type>>
T BaseNode::getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name) const
{
  T params;

  const auto list_elem = _sdf->FindElement(_name);
  if (!list_elem) {
    TOBAS_EXIT("Please specify '", _name, "'.");
  }

  auto item_elem = list_elem->FindElement("item");
  while (item_elem) {
    const auto value = item_elem->Get<typename T::value_type>();
    params.push_back(value);
    item_elem = item_elem->GetNextElement("item");
  }
  return params;
}

template <typename T>
  requires std::same_as<T, std::pair<typename T::first_type, typename T::first_type>>
T BaseNode::getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name) const
{
  const auto tmp = getSdfParam<gz::math::Vector2<typename T::first_type>>(_sdf, _name);
  return { tmp.X(), tmp.Y() };
}
}  // namespace gazebo
}  // namespace tobas
