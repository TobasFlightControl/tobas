#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/util.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

using namespace std;

class RotorAnomalyDetectorNode : public tobas::BaseNode
{
  using self = RotorAnomalyDetectorNode;
  using super = tobas::BaseNode;

  static constexpr rcl_duration_value_t kNoCommTimeout = 100'000'000;     // [ns]
  static constexpr rcl_duration_value_t kCommRecoveryTime = 500'000'000;  // [ns]
  static constexpr auto kPublishRotorLivelinessPeriod = 1s;

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
  map<string, RotorData> data_;  // Link Name -> RotorData

  ros2::PublisherPtr<tobas_msgs::msg::RotorLivelinessArray> rotor_liveliness_pub_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorStateArray> rotor_states_sub_;

  ros2::TimerPtr publish_rotor_liveliness_timer_;

  void publishRotorLiveliness();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void statesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& states);
};

RotorAnomalyDetectorNode::RotorAnomalyDetectorNode(const rclcpp::NodeOptions& options)
  : super("rotor_anomaly_detector", options)
{
  rotor_liveliness_pub_ = createPublisher<tobas_msgs::msg::RotorLivelinessArray>(tobas::kRotorLivelinessTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  rotor_states_sub_ = createSubscriber(tobas::addThrotNS(tobas::kRotorStatesTopic), &self::statesCb, this);

  publish_rotor_liveliness_timer_ = createTimer(kPublishRotorLivelinessPeriod, &self::publishRotorLiveliness, this);
}

void RotorAnomalyDetectorNode::publishRotorLiveliness()
{
  auto rotor_liveliness = std::make_unique<tobas_msgs::msg::RotorLivelinessArray>();
  rotor_liveliness->header.stamp = get_clock()->now();

  for (const auto& [link_name, data] : data_)
  {
    rotor_liveliness->data.emplace_back();
    rotor_liveliness->data.back().link_name = link_name;
    rotor_liveliness->data.back().alive = data.is_alive;
  }

  rotor_liveliness_pub_->publish(move(rotor_liveliness));
}

void RotorAnomalyDetectorNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  if (drone->prop == nullptr)
    return;

  drone_ = drone;
  data_.clear();

  // 電動推進系のみの機能
  if (drone->prop->type() != tobas::propulsion_system_t::ELECTRIC)
    return;

  for (const auto& [link_name, _] : drone->prop->rotors)
    data_[link_name] = RotorData();
}

void RotorAnomalyDetectorNode::statesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& states)
{
  if (drone_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Drone configuration is not received yet.");
    return;
  }

  for (const auto& state : states->states)
  {
    if (!data_.contains(state.link_name))
    {
      TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Invalid rotor: \"", state.link_name, "\"");
      continue;
    }

    auto& data = data_.at(state.link_name);
    const auto& cur_time = states->header.stamp;

    if (data.is_alive)
    {
      if (state.status == tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE)
      {
        // 一定時間通信が途絶えている場合は死んでいるとみなす
        if ((cur_time - data.last_alive_time).nanoseconds() > kNoCommTimeout)
        {
          data.is_alive = false;
          data.last_dead_time = cur_time;
          TOBAS_WARN("No communication with rotor \"", state.link_name, "\".");

          publishRotorLiveliness();
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
      if (state.status != tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE)
      {
        // 一定時間通信があれば回復したとみなす
        if ((cur_time - data.last_dead_time).nanoseconds() > kCommRecoveryTime)
        {
          data.is_alive = true;
          data.last_alive_time = cur_time;
          TOBAS_INFO("Communication with rotor \"", state.link_name, "\" has been recovered.");

          publishRotorLiveliness();
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
