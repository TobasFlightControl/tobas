#include <std_msgs/msg/bool.hpp>

#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/srv/disable_rotor.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

using namespace std;

class RotorCheckerNode : public tobas::BaseNode
{
  using self = RotorCheckerNode;
  using super = tobas::BaseNode;

  static constexpr rcl_duration_value_t kNoCommunicationTimeout = 20'000'000;  // [ns]

public:
  explicit RotorCheckerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  struct RotorData
  {
    bool is_alive = true;
    builtin_interfaces::msg::Time last_ok_time;
  };

  tobas::Drone::ConstSharedPtr drone_;
  std_msgs::msg::Bool::ConstSharedPtr arming_;
  std::map<size_t, RotorData> data_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorStateArray> states_sub_;

  ros2::ServiceClientPtr<tobas_msgs::srv::DisableRotor> remove_rotor_sc_;

  bool requestDisableRotor(uint8_t channel);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void statesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& states);
};

RotorCheckerNode::RotorCheckerNode(const rclcpp::NodeOptions& options) : super("rotor_checker", options)
{
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  states_sub_ = createSubscriber(path::join(tobas::kThrottledTopicNS, tobas::kRotorStatesTopic), &self::statesCb, this);

  remove_rotor_sc_ = create_client<tobas_msgs::srv::DisableRotor>(tobas::kRemoveRotorSrv);
}

bool RotorCheckerNode::requestDisableRotor(uint8_t channel)
{
  if (!remove_rotor_sc_->service_is_ready())
  {
    TOBAS_ERROR("\"", tobas::kRemoveRotorSrv, "\" service is not ready.");
    return false;
  }

  const auto req = std::make_shared<tobas_msgs::srv::DisableRotor::Request>();
  req->channel = channel;
  remove_rotor_sc_->async_send_request(req);

  return true;
}

void RotorCheckerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  for (const auto& rotor : drone->rotors)
    if (!data_.contains(rotor.channel))
      data_[rotor.channel] = RotorData();

  drone_ = drone;
}

void RotorCheckerNode::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void RotorCheckerNode::statesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& states)
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

    // 既に死んでいる場合はスキップ
    if (!data.is_alive)
      continue;

    if (state.status == tobas_msgs::msg::RotorState::NO_COMMUNICATION)
    {
      // 一定時間通信が途絶えている場合は死んでいるとみなす
      if ((cur_time - data.last_ok_time).nanoseconds() > kNoCommunicationTimeout)
      {
        data.is_alive = false;
        TOBAS_FATAL(
          "No communication with rotor channel ", (int)state.channel, ". Please land the drone as soon as possible.");

        // 通信できないモータをモデルから削除
        if (!requestDisableRotor(state.channel))
        {
          // TODO: Disarmしてパラシュートを開くなど
        }
      }
    }
    else
    {
      // 通信が確認できた最新の時刻を更新
      data.last_ok_time = cur_time;
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(RotorCheckerNode)
