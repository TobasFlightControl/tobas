// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/definitions.hpp>
#include <tobas_ros2_tools/qos.hpp>
#include <tobas_std_tools/stream.hpp>
#include <tobas_std_tools/vector.hpp>

#include <tobas_dparam_msgs/srv/get_params.hpp>
#include <tobas_msgs/msg/message.hpp>

#define TOBAS_EXIT(...)                                                                                                \
  {                                                                                                                    \
    TOBAS_FATAL(__VA_ARGS__);                                                                                          \
    rclcpp::shutdown();                                                                                                \
    abort();                                                                                                           \
  }

/** Assertion that also works in release mode. Use it to check operations that should almost never fail. */
#define TOBAS_ASSERT(expr)                                                                                             \
  {                                                                                                                    \
    if (!static_cast<bool>(expr)) {                                                                                    \
      TOBAS_FATAL("Assertion failed: ", __FILE__, ": ", __LINE__);                                                     \
      rclcpp::shutdown();                                                                                              \
      abort();                                                                                                         \
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

namespace tobas
{
class BaseNode : public rclcpp::Node
{
  using super = rclcpp::Node;
  using self = BaseNode;

  static constexpr long kMaxDynamicParamSteps = INT64_MAX;  // TODO: Limit to about 20 steps at most for easier tuning.

public:
  explicit BaseNode(const std::string& _node_name, const rclcpp::NodeOptions& _options);

  inline std::string ns() const;
  inline std::string name() const;

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
    Obj* _obj,
    rclcpp::CallbackGroup::SharedPtr _callback_group = nullptr);

  /**
   * Create an action server.
   *
   * @tparam ActType
   * @tparam Obj
   * @param _action_name
   * @param _handle_goal
   * @param _handle_cancel
   * @param _execute Action execution function that runs in a separate thread.
   * @param _obj
   */
  template <typename ActType, typename Obj>
  ros2::ActionServerPtr<ActType> createAction(
    const std::string& _action_name,
    rclcpp_action::GoalResponse (
      Obj::*_handle_goal)(const rclcpp_action::GoalUUID&, const std::shared_ptr<const typename ActType::Goal>&),
    rclcpp_action::CancelResponse (Obj::*_handle_cancel)(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>&),
    void (Obj::*_execute)(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>&),
    Obj* _obj);

  template <typename RepType, typename DurType, typename Obj>
  ros2::TimerPtr
  createTimer(std::chrono::duration<RepType, DurType> _period, void (Obj::*_fp)(void), Obj* _obj, bool _autostart = true);
  template <typename RepType, typename DurType, typename Obj>
  ros2::TimerPtr createWallTimer(
    std::chrono::duration<RepType, DurType> _period,
    void (Obj::*_fp)(void),
    Obj* _obj,
    bool _autostart = true);

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

  template <typename Obj>
  void addDynamicBoolParam(const std::string& _param_name, void (Obj::*_fp)(bool), Obj* _obj, bool _default_value);

  template <typename Obj>
  void addDynamicIntParam(
    const std::string& _param_name,
    void (Obj::*_fp)(long),
    Obj* _obj,
    long _step,
    long _default_value,
    long _minimum_value,
    long _maximum_value,
    const std::string& _prefix = "");

  template <typename Obj>
  void addDynamicDoubleParam(
    const std::string& _param_name,
    void (Obj::*_fp)(double),
    Obj* _obj,
    double _step,
    long _default_value,
    long _minimum_value,
    long _maximum_value,
    const std::string& _prefix = "");

  template <typename Obj>
  void addDynamicStringParam(
    const std::string& _param_name,
    void (Obj::*_fp)(const std::string&),
    Obj* _obj,
    const std::string& _default_value);

  bool getBoolParam(const std::string& _param_name);
  long getIntParam(const std::string& _param_name);
  double getDoubleParam(const std::string& _param_name);
  std::string getStringParam(const std::string& _param_name);
  std::vector<bool> getBoolArrayParam(const std::string& _param_name);
  std::vector<uint8_t> getByteArrayParam(const std::string& _param_name);
  std::vector<long> getIntArrayParam(const std::string& _param_name);
  std::vector<double> getDoubleArrayParam(const std::string& _param_name);
  std::vector<std::string> getStringArrayParam(const std::string& _param_name);

  bool getBoolParam(const std::string& _param_name, bool _default_value) noexcept;
  long getIntParam(const std::string& _param_name, long _default_value) noexcept;
  double getDoubleParam(const std::string& _param_name, double _default_value) noexcept;
  std::string getStringParam(const std::string& _param_name, const std::string& _default_value) noexcept;
  std::vector<bool> getBoolArrayParam(const std::string& _param_name, const std::vector<bool>& _default_value) noexcept;
  std::vector<uint8_t>
  getByteArrayParam(const std::string& _param_name, const std::vector<uint8_t>& _default_value) noexcept;
  std::vector<long> getIntArrayParam(const std::string& _param_name, const std::vector<long>& _default_value) noexcept;
  std::vector<double>
  getDoubleArrayParam(const std::string& _param_name, const std::vector<double>& _default_value) noexcept;
  std::vector<std::string>
  getStringArrayParam(const std::string& _param_name, const std::vector<std::string>& _default_value) noexcept;

  /** Tobas default node options. */
  static rclcpp::NodeOptions nodeOptions_Default(rclcpp::NodeOptions _options);

  /** Node options for nodes with dynamic parameters. */
  static rclcpp::NodeOptions nodeOptions_DParam(rclcpp::NodeOptions _options);

private:
  std::unordered_set<std::string> log_once_;
  std::unordered_map<std::string, rclcpp::Time> log_throttle_;

  ros2::PublisherPtr<tobas_msgs::msg::Message> message_pub_;

  tobas_dparam_msgs::msg::Parameters dparams_;  ///< Structure containing dynamic parameter settings and current values
  rclcpp::ParameterEventHandler dparam_sub_;
  std::vector<ros2::ParamHandlePtr> dparam_handles_;
  ros2::ServiceServerPtr<tobas_dparam_msgs::srv::GetParams> get_dparam_ss_;

  template <typename T>
  T declareParam(const std::string& _param_name);

  template <typename T>
  T declareParam(const std::string& _param_name, const T& _default_value);

  template <typename T>
  T declareDynamicParam(const std::string& _param_name, const T& _default_value);

  void rclcppLog(uint8_t _level, const std::string& _text) const;

  void getDParamCb(
    const tobas_dparam_msgs::srv::GetParams::Request::ConstSharedPtr& _req,
    const tobas_dparam_msgs::srv::GetParams::Response::SharedPtr& _res);

  static void setClockType(rclcpp::NodeOptions& _options);

  static inline std::string createID(const char* _file, int _line);
};

inline std::string BaseNode::ns() const
{
  return std::string(get_namespace());
}

inline std::string BaseNode::name() const
{
  return std::string(get_name());
}

template <typename MsgType>
ros2::PublisherPtr<MsgType>
BaseNode::createPublisher(const std::string& _topic_name, bool _latch, bool _reliable, size_t _queue_size)
{
  const ros2::qos::QoS qos(_latch, _reliable, _queue_size);
  return create_publisher<MsgType>(_topic_name, qos);
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
  const ros2::qos::QoS qos(_latch, _reliable, _queue_size);
  const auto cb = std::bind(_fp, _obj, std::placeholders::_1);
  return create_subscription<MsgType>(_topic_name, qos, cb);
}

template <typename SrvType, typename Obj>
ros2::ServiceServerPtr<SrvType> BaseNode::createService(
  const std::string& _srv_name,
  void (Obj::*_fp)(
    const std::shared_ptr<const typename SrvType::Request>&,
    const std::shared_ptr<typename SrvType::Response>&),
  Obj* _obj,
  rclcpp::CallbackGroup::SharedPtr _callback_group)
{
  auto cb = std::bind(_fp, _obj, std::placeholders::_1, std::placeholders::_2);
  return create_service<SrvType>(_srv_name, cb, rclcpp::ServicesQoS(), _callback_group);
}

template <typename ActType, typename Obj>
ros2::ActionServerPtr<ActType> BaseNode::createAction(
  const std::string& _action_name,
  rclcpp_action::GoalResponse (
    Obj::*_handle_goal)(const rclcpp_action::GoalUUID&, const std::shared_ptr<const typename ActType::Goal>&),
  rclcpp_action::CancelResponse (Obj::*_handle_cancel)(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>&),
  void (Obj::*_execute)(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>&),
  Obj* _obj)
{
  // Callback functions need to return quickly to avoid blocking the executor,
  // so we declare a lambda function to be called inside a new thread.
  const auto handle_accepted = [_execute, _obj](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& _gh)
  {
    const auto execute_in_thread = [_execute, _obj, _gh]() { return (_obj->*_execute)(_gh); };
    std::thread(execute_in_thread).detach();
  };

  return rclcpp_action::create_server<ActType>(
    _obj,
    _action_name,
    std::bind(_handle_goal, _obj, std::placeholders::_1, std::placeholders::_2),
    std::bind(_handle_cancel, _obj, std::placeholders::_1),
    handle_accepted);
}

template <typename RepType, typename DurType, typename Obj>
ros2::TimerPtr
BaseNode::createTimer(std::chrono::duration<RepType, DurType> _period, void (Obj::*_fp)(void), Obj* _obj, bool _autostart)
{
  const auto timer = create_timer(_period, bind(_fp, _obj));

  if (!_autostart) {
    timer->cancel();
  }

  return timer;
}

template <typename RepType, typename DurType, typename Obj>
ros2::TimerPtr BaseNode::createWallTimer(
  std::chrono::duration<RepType, DurType> _period,
  void (Obj::*_fp)(void),
  Obj* _obj,
  bool _autostart)
{
  return create_wall_timer(_period, bind(_fp, _obj), nullptr, _autostart);
}

template <typename Obj>
void BaseNode::addDynamicBoolParam(const std::string& _param_name, void (Obj::*_fp)(bool), Obj* _obj, bool _default_value)
{
  if (has_parameter(_param_name)) {
    TOBAS_ERROR("Parameter '", _param_name, "' is already declared.");
    return;
  }

  const auto initial_value = declareDynamicParam(_param_name, _default_value);

  tobas_dparam_msgs::msg::BoolParam dparam;
  dparam.name = _param_name;
  dparam.default_value = _default_value;
  dparam.initial_value = initial_value;
  dparams_.bools.push_back(dparam);

  const auto cb = [this, _param_name, _fp, _obj](const rclcpp::Parameter& _param)
  {
    const auto value = _param.as_bool();
    (_obj->*_fp)(value);
    for (auto& bool_param : dparams_.bools) {
      if (bool_param.name == _param_name) {
        bool_param.current_value = value;
        break;
      }
    }
    TOBAS_INFO("Boolean parameter '", _param_name, "' has been updated to ", value, ".");
  };
  const auto cb_handle = dparam_sub_.add_parameter_callback(_param_name, cb);
  dparam_handles_.push_back(cb_handle);
}

template <typename Obj>
void BaseNode::addDynamicIntParam(
  const std::string& _param_name,
  void (Obj::*_fp)(long),
  Obj* _obj,
  long _step,
  long _default_value,
  long _minimum_value,
  long _maximum_value,
  const std::string& _prefix)
{
  TOBAS_ASSERT(_minimum_value <= _default_value && _default_value <= _maximum_value);
  TOBAS_ASSERT(_maximum_value - _minimum_value < kMaxDynamicParamSteps);

  if (has_parameter(_param_name)) {
    TOBAS_ERROR("Parameter '", _param_name, "' is already declared.");
    return;
  }

  const auto initial_value = declareDynamicParam(_param_name, _default_value);

  tobas_dparam_msgs::msg::IntParam dparam;
  dparam.name = _param_name;
  dparam.step = _step;
  dparam.default_value = _default_value;
  dparam.initial_value = initial_value;
  dparam.minimum_value = _minimum_value;
  dparam.maximum_value = _maximum_value;
  dparam.prefix = _prefix;
  dparams_.ints.push_back(dparam);

  const auto cb =
    [this, _param_name, _fp, _obj, _step, _minimum_value, _maximum_value, _prefix](const rclcpp::Parameter& _param)
  {
    const auto lsb = std::clamp(_param.as_int(), _minimum_value, _maximum_value);
    const auto value = _step * lsb;
    (_obj->*_fp)(value);
    for (auto& int_param : dparams_.ints) {
      if (int_param.name == _param_name) {
        int_param.current_value = lsb;
        break;
      }
    }
    TOBAS_INFO("Integer parameter '", _param_name, "' has been updated to ", value, _prefix, ".");
  };
  const auto cb_handle = dparam_sub_.add_parameter_callback(_param_name, cb);
  dparam_handles_.push_back(cb_handle);
}

template <typename Obj>
void BaseNode::addDynamicDoubleParam(
  const std::string& _param_name,
  void (Obj::*_fp)(double),
  Obj* _obj,
  double _step,
  long _default_value,
  long _minimum_value,
  long _maximum_value,
  const std::string& _prefix)
{
  TOBAS_ASSERT(_minimum_value <= _default_value && _default_value <= _maximum_value);
  TOBAS_ASSERT(_maximum_value - _minimum_value < kMaxDynamicParamSteps);

  if (has_parameter(_param_name)) {
    TOBAS_ERROR("Parameter '", _param_name, "' is already declared.");
    return;
  }

  const auto initial_value = declareDynamicParam(_param_name, _default_value);

  tobas_dparam_msgs::msg::DoubleParam dparam;
  dparam.name = _param_name;
  dparam.step = _step;
  dparam.default_value = _default_value;
  dparam.initial_value = initial_value;
  dparam.minimum_value = _minimum_value;
  dparam.maximum_value = _maximum_value;
  dparam.prefix = _prefix;
  dparams_.doubles.push_back(dparam);

  const auto cb =
    [this, _param_name, _fp, _obj, _step, _minimum_value, _maximum_value, _prefix](const rclcpp::Parameter& _param)
  {
    const auto lsb = std::clamp(_param.as_int(), _minimum_value, _maximum_value);
    const auto value = _step * lsb;
    (_obj->*_fp)(value);
    for (auto& double_param : dparams_.doubles) {
      if (double_param.name == _param_name) {
        double_param.current_value = lsb;
        break;
      }
    }
    TOBAS_INFO("Double parameter '", _param_name, "' has been updated to ", value, _prefix, ".");
  };
  const auto cb_handle = dparam_sub_.add_parameter_callback(_param_name, cb);
  dparam_handles_.push_back(cb_handle);
}

template <typename Obj>
void BaseNode::addDynamicStringParam(
  const std::string& _param_name,
  void (Obj::*_fp)(const std::string&),
  Obj* _obj,
  const std::string& _default_value)
{
  if (has_parameter(_param_name)) {
    TOBAS_ERROR("Parameter '", _param_name, "' is already declared.");
    return;
  }

  const auto initial_value = declareDynamicParam(_param_name, _default_value);

  tobas_dparam_msgs::msg::StringParam dparam;
  dparam.name = _param_name;
  dparam.default_value = _default_value;
  dparam.initial_value = initial_value;
  dparams_.strings.push_back(dparam);

  const auto cb = [this, _param_name, _fp, _obj](const rclcpp::Parameter& _param)
  {
    const auto& value = _param.as_string();
    (_obj->*_fp)(value);
    for (auto& string_param : dparams_.strings) {
      if (string_param.name == _param_name) {
        string_param.current_value = value;
        break;
      }
    }
    TOBAS_INFO("String parameter '", _param_name, "' has been updated to '", value, "'.");
  };
  const auto cb_handle = dparam_sub_.add_parameter_callback(_param_name, cb);
  dparam_handles_.push_back(cb_handle);
}

template <typename... Args>
void BaseNode::log(uint8_t _level, const Args&... _args) const
{
  // Create message.
  auto message = std::make_unique<tobas_msgs::msg::Message>();
  message->header.stamp = now();
  message->level = _level;
  message->name = get_name();
  message->message = st::buildString(_args...);

  // Output message to the console
  rclcppLog(_level, message->message);

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
  const auto cur_time = now();
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
T BaseNode::declareParam(const std::string& _param_name)
{
  try {
    return declare_parameter<T>(_param_name);
  }
  catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException&) {
    TOBAS_EXIT("Parameter '", _param_name, "' is not initialized.");
  }
  catch (const std::exception& e) {
    TOBAS_EXIT("Unexptected error while declaring '", _param_name, "': ", e.what());
  }
}

template <typename T>
T BaseNode::declareParam(const std::string& _param_name, const T& _default_value)
{
  try {
    return declare_parameter<T>(_param_name);
  }
  catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException&) {
    TOBAS_WARN("Parameter '", _param_name, "' is not initialized. The default value '", _default_value, "' is set.");

    // At this point the parameter has only been declared, so set the default value.
    const auto set_param_res = set_parameter(rclcpp::Parameter(_param_name, _default_value));
    if (!set_param_res.successful) {
      TOBAS_ERROR("Failed to set '", _param_name, "': ", set_param_res.reason);
    }

    return _default_value;
  }
  catch (const std::exception& e) {
    TOBAS_EXIT("Unexptected error while declaring '", _param_name, "': ", e.what());
  }
}

template <typename T>
T BaseNode::declareDynamicParam(const std::string& _name, const T& _default_value)
{
  if (!get_dparam_ss_) {
    get_dparam_ss_ = createService<tobas_dparam_msgs::srv::GetParams>(
      name() + "/" + service::kGetDynamicParams, &self::getDParamCb, this);
  }

  return declare_parameter(_name, _default_value);
}

inline std::string BaseNode::createID(const char* _file, int _line)
{
  return std::string(_file) + ":" + std::to_string(_line);
}
}  // namespace tobas
