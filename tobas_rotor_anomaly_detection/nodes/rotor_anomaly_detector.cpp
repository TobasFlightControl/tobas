#include <std_msgs/msg/bool.hpp>

#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/util.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/srv/enable_rotor.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

class RotorAnomalyDetectorNode : public tobas::BaseNode
{
  using self = RotorAnomalyDetectorNode;
  using super = tobas::BaseNode;

  static constexpr rcl_duration_value_t kNoCommTimeout = 100'000'000;     // [ns]
  static constexpr rcl_duration_value_t kCommRecoveryTime = 500'000'000;  // [ns]

public:
  explicit RotorAnomalyDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  struct RotorData
  {
    bool is_alive = true;
    builtin_interfaces::msg::Time last_alive_time;
    builtin_interfaces::msg::Time last_dead_time;
  };

  tobas::Drone::ConstSharedPtr drone_;
  std_msgs::msg::Bool::ConstSharedPtr arming_;
  std::map<size_t, RotorData> data_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorStateArray> states_sub_;

  ros2::ServiceClientPtr<tobas_msgs::srv::EnableRotor> enable_rotor_sc_;

  void requestEnableRotor(uint8_t channel, bool enable);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void statesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& states);
};

RotorAnomalyDetectorNode::RotorAnomalyDetectorNode(const rclcpp::NodeOptions& options)
  : super("rotor_anomaly_detector", options)
{
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  states_sub_ = createSubscriber(tobas::addThrotNS(tobas::kRotorStatesTopic), &self::statesCb, this);

  enable_rotor_sc_ = create_client<tobas_msgs::srv::EnableRotor>(tobas::kEnableRotorSrv);
}

void RotorAnomalyDetectorNode::requestEnableRotor(uint8_t channel, bool enable)
{
  if (!enable_rotor_sc_->service_is_ready())
  {
    TOBAS_ERROR("\"", tobas::kEnableRotorSrv, "\" service is not ready.");
    return;
  }

  const auto req = std::make_shared<tobas_msgs::srv::EnableRotor::Request>();
  req->channel = channel;
  req->enable = enable;
  enable_rotor_sc_->async_send_request(req);
}

void RotorAnomalyDetectorNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  for (const auto& [channel, _] : drone->rotors)
    if (!data_.contains(channel))
      data_[channel] = RotorData();

  drone_ = drone;
}

void RotorAnomalyDetectorNode::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void RotorAnomalyDetectorNode::statesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& states)
{
  if (drone_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Drone configuration is not received yet.");
    return;
  }
  if (arming_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Arming status is not received yet.");
    return;
  }

  if (!arming_->data)
    return;

  for (const auto& state : states->states)
  {
    if (!data_.contains(state.channel))
    {
      TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Invalid rotor channel: ", (int)state.channel);
      continue;
    }

    auto& data = data_.at(state.channel);
    const auto& cur_time = states->header.stamp;

    if (data.is_alive)
    {
      if (state.status == tobas_msgs::msg::RotorState::NO_COMMUNICATION)
      {
        // 一定時間通信が途絶えている場合は死んでいるとみなす
        if ((cur_time - data.last_alive_time).nanoseconds() > kNoCommTimeout)
        {
          data.is_alive = false;
          data.last_dead_time = cur_time;
          TOBAS_FATAL(
            "No communication with rotor channel ", (int)state.channel, ". Please land the drone as soon as possible.");

          // 通信できないモータをモデルから削除
          requestEnableRotor(state.channel, false);
        }
      }
      else
      {
        // 通信が確認できた最新の時刻を更新
        data.last_alive_time = cur_time;
      }
    }
    else
    {
      if (state.status != tobas_msgs::msg::RotorState::NO_COMMUNICATION)
      {
        // 一定時間通信があれば回復したとみなす
        if ((cur_time - data.last_dead_time).nanoseconds() > kCommRecoveryTime)
        {
          data.is_alive = true;
          data.last_alive_time = cur_time;
          TOBAS_INFO("Communication with rotor channel ", (int)state.channel, " has been recovered.");

          // 回復したモータをモデルに追加
          requestEnableRotor(state.channel, true);
        }
      }
      else
      {
        // 通信が確認できない最新の時刻を更新
        data.last_dead_time = cur_time;
      }
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(RotorAnomalyDetectorNode)
