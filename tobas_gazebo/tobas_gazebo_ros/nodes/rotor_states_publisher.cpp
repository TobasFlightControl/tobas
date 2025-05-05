#include <tobas_path_tools/join.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_gazebo_common/constants.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>

class RotorStatesPublisherNode : public tobas::BaseNode
{
  using self = RotorStatesPublisherNode;
  using super = tobas::BaseNode;

public:
  explicit RotorStatesPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone::ConstSharedPtr drone_;
  std::map<std::string, tobas_msgs::msg::RotorState> rotor_states_;

  ros2::PublisherPtr<tobas_msgs::msg::RotorStateArray> rotor_states_pub_;
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  std::map<std::string, ros2::SubscriberPtr<tobas_msgs::msg::RotorState>> rotor_state_subs_;

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
  if (!drone->prop) {
    return;
  }

  rotor_states_.clear();
  rotor_state_subs_.clear();

  for (const auto& [link_name, _] : drone->prop->rotors) {
    const auto topic = path::join(gazebo::kRotorStateTopicNS, link_name);
    rotor_state_subs_[link_name] = createSubscriber(topic, &self::rotorStateCb, this);
  }

  drone_ = drone;
}

void RotorStatesPublisherNode::rotorStateCb(const tobas_msgs::msg::RotorState::ConstSharedPtr& rotor_state)
{
  const auto& link_name = rotor_state->link_name;

  if (rotor_states_.contains(link_name)) {
    TOBAS_WARN("Rotor \"", link_name, "\" is already updated.");
    return;
  }

  // Store rotor state
  rotor_states_[link_name] = *rotor_state;

  if (rotor_states_.size() == drone_->prop->numRotors()) {
    // Publish rotor states
    auto rotor_states_msg = std::make_unique<tobas_msgs::msg::RotorStateArray>();
    rotor_states_msg->header.stamp = get_clock()->now();
    for (const auto& [_, state] : rotor_states_) {
      rotor_states_msg->states.push_back(state);
    }
    rotor_states_pub_->publish(move(rotor_states_msg));

    // Reset
    rotor_states_.clear();
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(RotorStatesPublisherNode)
