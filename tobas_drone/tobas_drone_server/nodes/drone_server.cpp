#include <std_msgs/msg/bool.hpp>

#include <tobas_path_tools/join.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

class DroneServerNode : public tobas::BaseNode
{
  using self = DroneServerNode;
  using super = tobas::BaseNode;

public:
  explicit DroneServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;
  std_msgs::msg::Bool::ConstSharedPtr arming_;

  ros2::PublisherPtr<tobas::Drone> drone_pub_;
  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorStateArray> rotor_states_sub_;

  bool fileParamCb(const std::string& p);

  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& rotor_states);
};

DroneServerNode::DroneServerNode(const rclcpp::NodeOptions& options) : super("drone_server", options)
{
  addDynamicStringParam("tbsdrn_path", &self::fileParamCb, this);

  drone_pub_ = createPublisher<tobas::Drone>(tobas::kDroneTopic, true, true);

  arming_sub_ = createSubscriber<std_msgs::msg::Bool>(tobas::kArmingTopic, &self::armingCb, this);
  rotor_states_sub_ = createSubscriber<tobas_msgs::msg::RotorStateArray>(
    path::join(tobas::kThrottledTopicNS, tobas::kRotorStatesTopic), &self::rotorStatesCb, this);
}

bool DroneServerNode::fileParamCb(const std::string& p)
{
  // Load drone configuration
  if (!drone_.load(p))
  {
    TOBAS_ERROR("Failed to load drone configurations from \"", p, "\".");
    return false;
  }

  // Check drone configuration validity
  if (!drone_.isValid())
  {
    TOBAS_ERROR("Drone configurations are invalid.");
    return false;
  }

  // Publish drone configuration
  auto drone_msg = std::make_unique<tobas::Drone>(drone_);
  drone_pub_->publish(move(drone_msg));

  TOBAS_INFO("New drone configuration message is published.");
  return true;
}

void DroneServerNode::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void DroneServerNode::rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& rotor_states)
{
  if (arming_ == nullptr || !arming_->data)
    return;

  (void)rotor_states;  // TODO
}

RCLCPP_COMPONENTS_REGISTER_NODE(DroneServerNode)
