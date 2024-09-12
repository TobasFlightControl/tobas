#pragma once

#include <rclcpp_components/register_node_macro.hpp>

#include <tobas_std_tools/stream.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_ros2_tools/definitions.hpp>
#include <tobas_ros2_tools/qos.hpp>
#include <tobas_std_msgs/msg/message.hpp>
#include <tobas_dparam_msgs/msg/parameters.hpp>

#define TOBAS_EXIT(...)                                                                                                \
  {                                                                                                                    \
    TOBAS_FATAL(__VA_ARGS__);                                                                                          \
    throw;                                                                                                             \
  }

/* リリースモードでも機能するアサーション．ほとんど失敗し得ない操作の成否を一応確認するために使う． */
#define TOBAS_ASSERT(expr)                                                                                             \
  {                                                                                                                    \
    if (!static_cast<bool>(expr))                                                                                      \
    {                                                                                                                  \
      TOBAS_FATAL("Assertion failed: ", __FILE__, ": ", __LINE__);                                                     \
      throw;                                                                                                           \
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

public:
  explicit BaseNode(const std::string& node_name, const rclcpp::NodeOptions& options);

  inline std::string ns() const;
  inline std::string name() const;

  template <typename MsgType>
  ros2::PublisherPtr<MsgType> createPublisher(
    const std::string& topic_name,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename MsgType, typename Obj>
  ros2::SubscriberPtr<MsgType> createSubscriber(
    const std::string& topic_name,
    void (Obj::*fp)(const std::shared_ptr<const MsgType>&),
    Obj* obj,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename SrvType, typename Obj>
  ros2::ServiceServerPtr<SrvType> createService(
    const std::string& srv_name,
    void (Obj::*fp)(
      const std::shared_ptr<const typename SrvType::Request>&,
      const std::shared_ptr<typename SrvType::Response>&),
    Obj* obj);

  /**
   * @brief アクションサーバを作成する．
   *
   * @tparam ActionType
   * @tparam Obj
   * @param action_name
   * @param handle_goal
   * @param handle_cancel
   * @param execute 別スレッドで実行されるアクションの実行関数
   * @param obj
   * @return ros2::ActionServerPtr<ActionType>
   */
  template <typename ActionType, typename Obj>
  ros2::ActionServerPtr<ActionType> createAction(
    const std::string& action_name,
    rclcpp_action::GoalResponse (
      Obj::*handle_goal)(const rclcpp_action::GoalUUID&, std::shared_ptr<const typename ActionType::Goal>),
    rclcpp_action::CancelResponse (Obj::*handle_cancel)(ros2::ActionGoalHandlePtr<ActionType>),
    void (Obj::*execute)(ros2::ActionGoalHandlePtr<ActionType>),
    Obj* obj);

  template <typename RepType, typename DurType, typename Obj>
  ros2::TimerPtr createTimer(std::chrono::duration<RepType, DurType> period, void (Obj::*fp)(void), Obj* obj);

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

  template <typename Obj>
  void
  addDynamicBoolParam(const std::string& name, bool (Obj::*fp)(const bool&), Obj* obj, const bool& _default = false);

  template <typename Obj>
  void addDynamicIntParam(
    const std::string& name,
    bool (Obj::*fp)(const long&),
    Obj* obj,
    const long& _default = 0,
    const long& _min = std::numeric_limits<long>::lowest(),
    const long& _max = std::numeric_limits<long>::max());

  template <typename Obj>
  void addDynamicDoubleParam(
    const std::string& name,
    bool (Obj::*fp)(const double&),
    Obj* obj,
    const double& _default = 0.,
    const double& _min = std::numeric_limits<double>::lowest(),
    const double& _max = std::numeric_limits<double>::max());

  template <typename Obj>
  void addDynamicStringParam(
    const std::string& name,
    bool (Obj::*fp)(const std::string&),
    Obj* obj,
    const std::string& _default = "");

  void publishDynamicParameterDescriptions();

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

  ros2::PublisherPtr<tobas_std_msgs::msg::Message> message_pub_;

  tobas_dparam_msgs::msg::Parameters dparams_;
  ros2::PublisherPtr<tobas_dparam_msgs::msg::Parameters> dparams_pub_;
  rclcpp::ParameterEventHandler dparam_sub_;
  std::vector<ros2::ParamHandlePtr> dparam_handles_;

  template <typename T>
  T declareParam(const std::string& name);

  template <typename T>
  T declareParam(const std::string& name, const T& _default);

  void rclcppLog(uint8_t level, const std::string& text) const;

  static std::string createID(const char* file, int line);
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
BaseNode::createPublisher(const std::string& topic_name, bool latch, bool reliable, size_t queue_size)
{
  return create_publisher<MsgType>(topic_name, ros2::makeQoS(latch, reliable, queue_size));
}

template <typename MsgType, typename Obj>
ros2::SubscriberPtr<MsgType> BaseNode::createSubscriber(
  const std::string& topic_name,
  void (Obj::*fp)(const std::shared_ptr<const MsgType>&),
  Obj* obj,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  return create_subscription<MsgType>(
    topic_name, ros2::makeQoS(latch, reliable, queue_size), std::bind(fp, obj, std::placeholders::_1));
}

template <typename SrvType, typename Obj>
ros2::ServiceServerPtr<SrvType> BaseNode::createService(
  const std::string& srv_name,
  void (Obj::*fp)(
    const std::shared_ptr<const typename SrvType::Request>&,
    const std::shared_ptr<typename SrvType::Response>&),
  Obj* obj)
{
  return create_service<SrvType>(srv_name, std::bind(fp, obj, std::placeholders::_1, std::placeholders::_2));
}

template <typename ActionType, typename Obj>
ros2::ActionServerPtr<ActionType> BaseNode::createAction(
  const std::string& action_name,
  rclcpp_action::GoalResponse (
    Obj::*handle_goal)(const rclcpp_action::GoalUUID&, std::shared_ptr<const typename ActionType::Goal>),
  rclcpp_action::CancelResponse (Obj::*handle_cancel)(ros2::ActionGoalHandlePtr<ActionType>),
  void (Obj::*execute)(ros2::ActionGoalHandlePtr<ActionType>),
  Obj* obj)
{
  // Callback functions need to return quickly to avoid blocking the executor,
  // so we declare a lambda function to be called inside a new thread.
  auto handle_accepted = [execute, obj](ros2::ActionGoalHandlePtr<ActionType> goal_handle)
  {
    auto execute_in_thread = [execute, obj, goal_handle]() { return (obj->*execute)(goal_handle); };
    std::thread(execute_in_thread).detach();
  };

  return rclcpp_action::create_server<ActionType>(
    obj, action_name, std::bind(handle_goal, obj, std::placeholders::_1, std::placeholders::_2),
    std::bind(handle_cancel, obj, std::placeholders::_1), handle_accepted);
}

template <typename RepType, typename DurType, typename Obj>
ros2::TimerPtr BaseNode::createTimer(std::chrono::duration<RepType, DurType> period, void (Obj::*fp)(void), Obj* obj)
{
  return create_timer(period, bind(fp, obj));
}

template <typename Obj>
void BaseNode::addDynamicBoolParam(
  const std::string& name,
  bool (Obj::*fp)(const bool&),
  Obj* obj,
  const bool& _default)
{
  if (has_parameter(name))
  {
    TOBAS_ERROR("Parameter \"", name, "\" is already declared.");
    return;
  }

  declare_parameter(name, _default);

  const auto cb = [this, name, fp, obj](const rclcpp::Parameter& param)
  {
    const auto value = param.as_bool();
    if ((obj->*fp)(value))
      TOBAS_INFO("Boolean parameter \"", name, "\" is updated successfully.");
  };
  const auto cb_handle = dparam_sub_.add_parameter_callback(name, cb);
  dparam_handles_.push_back(cb_handle);

  tobas_dparam_msgs::msg::BoolParam bool_param;
  bool_param.name = name;
  bool_param.dflt = _default;
  dparams_.bools.push_back(bool_param);
}

template <typename Obj>
void BaseNode::addDynamicIntParam(
  const std::string& name,
  bool (Obj::*fp)(const long&),
  Obj* obj,
  const long& _default,
  const long& _min,
  const long& _max)
{
  assert(_min <= _default && _default <= _max);

  if (has_parameter(name))
  {
    TOBAS_ERROR("Parameter \"", name, "\" is already declared.");
    return;
  }

  declare_parameter(name, _default);

  const auto cb = [this, name, fp, obj, _min, _max](const rclcpp::Parameter& param)
  {
    auto value = param.as_int();
    if (value < _min || _max < value)
    {
      value = std::clamp(value, _min, _max);
      TOBAS_WARN(
        "You attempted to set \"", name, "\" to ", value, ", but since it was outside the range [", _min, ", ", _max,
        "], it was clamped to ", value, ".");
    }
    if ((obj->*fp)(value))
      TOBAS_INFO("Integer parameter \"", name, "\" is updated successfully.");
  };
  const auto cb_handle = dparam_sub_.add_parameter_callback(name, cb);
  dparam_handles_.push_back(cb_handle);

  tobas_dparam_msgs::msg::IntParam int_param;
  int_param.name = name;
  int_param.dflt = _default;
  int_param.min = _min;
  int_param.max = _max;
  dparams_.ints.push_back(int_param);
}

template <typename Obj>
void BaseNode::addDynamicDoubleParam(
  const std::string& name,
  bool (Obj::*fp)(const double&),
  Obj* obj,
  const double& _default,
  const double& _min,
  const double& _max)
{
  assert(_min <= _default && _default <= _max);

  if (has_parameter(name))
  {
    TOBAS_ERROR("Parameter \"", name, "\" is already declared.");
    return;
  }

  declare_parameter(name, _default);

  const auto cb = [this, name, fp, obj, _min, _max](const rclcpp::Parameter& param)
  {
    auto value = param.as_double();
    if (value < _min || _max < value)
    {
      value = std::clamp(value, _min, _max);
      TOBAS_WARN(
        "You attempted to set \"", name, "\" to ", value, ", but since it was outside the range [", _min, ", ", _max,
        "], it was clamped to ", value, ".");
    }
    if ((obj->*fp)(value))
      TOBAS_INFO("Double parameter \"", name, "\" is updated successfully.");
  };
  const auto cb_handle = dparam_sub_.add_parameter_callback(name, cb);
  dparam_handles_.push_back(cb_handle);

  tobas_dparam_msgs::msg::DoubleParam double_param;
  double_param.name = name;
  double_param.dflt = _default;
  double_param.min = _min;
  double_param.max = _max;
  dparams_.doubles.push_back(double_param);
}

template <typename Obj>
void BaseNode::addDynamicStringParam(
  const std::string& name,
  bool (Obj::*fp)(const std::string&),
  Obj* obj,
  const std::string& _default)
{
  if (has_parameter(name))
  {
    TOBAS_ERROR("Parameter \"", name, "\" is already declared.");
    return;
  }

  declare_parameter(name, _default);

  const auto cb = [this, name, fp, obj](const rclcpp::Parameter& param)
  {
    const auto& value = param.as_string();
    if ((obj->*fp)(value))
      TOBAS_INFO("String parameter \"", name, "\" is updated successfully.");
  };
  const auto cb_handle = dparam_sub_.add_parameter_callback(name, cb);
  dparam_handles_.push_back(cb_handle);

  tobas_dparam_msgs::msg::StringParam string_param;
  string_param.name = name;
  string_param.dflt = _default;
  dparams_.strings.push_back(string_param);
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

template <typename T>
T BaseNode::declareParam(const std::string& name)
{
  try
  {
    return declare_parameter<T>(name);
  }
  catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException& e)
  {
    TOBAS_EXIT(e.what());
  }
}

template <typename T>
T BaseNode::declareParam(const std::string& name, const T& _default)
{
  try
  {
    return declare_parameter<T>(name);
  }
  catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException&)
  {
    TOBAS_WARN("Parameter \"", name, "\" is not initialized. The default value \"", _default, "\" is set.");

    // この時点で宣言だけは済んでいるので，デフォルト値をセットする．
    const auto set_param_res = set_parameter(rclcpp::Parameter(name, _default));
    if (!set_param_res.successful)
      TOBAS_ERROR("Failed to set \"", name, "\": ", set_param_res.reason);

    return _default;
  }
}
}  // namespace tobas
