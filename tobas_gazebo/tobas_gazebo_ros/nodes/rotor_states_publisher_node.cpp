#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>

#include <tobas_gazebo_common/constants.hpp>

using namespace std;

class RotorStatesPublisherNode : public tobas::BaseNode
{
  using self = RotorStatesPublisherNode;
  using super = tobas::BaseNode;

public:
  explicit RotorStatesPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone::ConstSharedPtr drone_;
  map<size_t, bool> is_updated_;
  std::vector<tobas_msgs::msg::RotorState> rotor_states_;

  ros2::PublisherPtr<tobas_msgs::msg::RotorStateArray> rotor_states_pub_;
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  map<size_t, ros2::SubscriberPtr<tobas_msgs::msg::RotorState>> rotor_state_subs_;

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void rotorStateCb(const tobas_msgs::msg::RotorState::ConstSharedPtr& rotor_state);
};

RotorStatesPublisherNode::RotorStatesPublisherNode(const rclcpp::NodeOptions& options)
  : super("gazebo_rotor_states_publisher", options)
{
  rotor_states_pub_ = createPublisher<tobas_msgs::msg::RotorStateArray>(tobas::kRotorStatesTopic);
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
}

void RotorStatesPublisherNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  is_updated_.clear();
  rotor_states_.clear();
  rotor_state_subs_.clear();

  for (const auto& rotor : drone->rotors)
  {
    is_updated_[rotor.channel] = false;

    const auto topic = gazebo::kRotorStateTopicPrefix + to_string(rotor.channel);
    rotor_state_subs_[rotor.channel] = createSubscriber(topic, &self::rotorStateCb, this);
  }

  drone_ = drone;

  TOBAS_INFO("Gazego rotor states publisher is initialized.");
}

void RotorStatesPublisherNode::rotorStateCb(const tobas_msgs::msg::RotorState::ConstSharedPtr& rotor_state)
{
  const auto& channel = rotor_state->channel;

  if (!is_updated_.contains(channel))
  {
    TOBAS_ERROR("Invalid rotor channel: ", channel);
    return;
  }

  if (is_updated_[channel])
  {
    TOBAS_WARN("Rotor channel ", (int)channel, " is already updated.");
    return;
  }

  rotor_states_.push_back(*rotor_state);
  is_updated_[channel] = true;

  if (rotor_states_.size() == drone_->numRotors())
  {
    // Publish rotor states
    auto rotor_states_msg = std::make_unique<tobas_msgs::msg::RotorStateArray>();
    rotor_states_msg->header.stamp = get_clock()->now();
    rotor_states_msg->states = rotor_states_;
    rotor_states_pub_->publish(move(rotor_states_msg));

    // Reset
    for (auto& [_, is_updated] : is_updated_)
      is_updated = false;
    rotor_states_.clear();
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(RotorStatesPublisherNode)
