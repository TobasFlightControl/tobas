#include <std_msgs/msg/bool.hpp>

#include <tobas_path_tools/join.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>

#include <tobas_kdl_msgs/msg/euler_stamped.hpp>
#include <tobas_std_msgs/msg/message.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/gps.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>
#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>
#include <tobas_hal_msgs/msg/adc.hpp>
#include <tobas_hal_msgs/msg/sbus.hpp>
#include <tobas_hal_msgs/msg/imu.hpp>
#include <tobas_hal_msgs/msg/magnetic_field.hpp>
#include <tobas_hal_msgs/msg/fluid_pressure.hpp>

using namespace std;

class ROSInterfaceNode : public tobas::BaseNode
{
  using self = ROSInterfaceNode;
  using super = tobas::BaseNode;

public:
  explicit ROSInterfaceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // PubSub
  map<string, rclcpp::PublisherBase::SharedPtr> pubs_;
  map<string, rclcpp::SubscriptionBase::SharedPtr> subs_;

  template <typename MsgType>
  void addTopicRemoteToLocal(
    const string& sub_topic,
    const string& pub_topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename MsgType>
  void topicCallback(const typename MsgType::ConstSharedPtr& msg_in, const string& pub_topic);

  static string throttled(const std::string& topic);
};

ROSInterfaceNode::ROSInterfaceNode(const rclcpp::NodeOptions& options) : super("ros_interface", options)
{
  addTopicRemoteToLocal<tobas_std_msgs::msg::Message>(tobas::kMessageTopic, tobas::kMessageTopic);
  addTopicRemoteToLocal<tobas_msgs::msg::Battery>(throttled(tobas::kBatteryLpfTopic), tobas::kBatteryTopic);
  addTopicRemoteToLocal<tobas_msgs::msg::Cpu>(tobas::kCPUTopic, tobas::kCPUTopic);
  addTopicRemoteToLocal<tobas_msgs::msg::RCInput>(throttled(tobas::kRcInputTopic), tobas::kRcInputTopic);
  addTopicRemoteToLocal<tobas_msgs::msg::Gps>(tobas::kGNSSTopic, tobas::kGNSSTopic);
  addTopicRemoteToLocal<tobas_msgs::msg::RotorSpeeds>(throttled(tobas::kRotorSpeedsTopic), tobas::kRotorSpeedsTopic);
  addTopicRemoteToLocal<tobas_kdl_msgs::msg::Euler>(tobas::kEulerTopic, tobas::kEulerTopic);
  addTopicRemoteToLocal<std_msgs::msg::Bool>(tobas::kArmingTopic, tobas::kArmingTopic);
  addTopicRemoteToLocal<tobas_msgs::msg::PreArmCheck>(tobas::kPreArmCheckTopic, tobas::kPreArmCheckTopic);
  addTopicRemoteToLocal<tobas_hal_msgs::msg::Adc>(hal::kADCTopic, hal::kADCTopic);
  addTopicRemoteToLocal<tobas_hal_msgs::msg::Sbus>(hal::kSBUSTopic, hal::kSBUSTopic);
  addTopicRemoteToLocal<tobas_hal_msgs::msg::Imu>(hal::kIMUTopic, hal::kIMUTopic);
  addTopicRemoteToLocal<tobas_hal_msgs::msg::MagneticField>(hal::kMagTopic, hal::kMagTopic);
  addTopicRemoteToLocal<tobas_hal_msgs::msg::FluidPressure>(hal::kAirPressureTopic, hal::kAirPressureTopic);
}

template <typename MsgType>
void ROSInterfaceNode::addTopicRemoteToLocal(
  const string& sub_topic,
  const string& pub_topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  const auto qos = ros2::makeQoS(latch, reliable, queue_size);
  const auto pub_topic_if = path::join(tobas::kInterfaceTopicPrefix, pub_topic);

  const auto cb = [this, pub_topic_if](const typename MsgType::ConstSharedPtr& msg)
  { topicCallback<MsgType>(msg, pub_topic_if); };
  subs_[sub_topic] = create_subscription<MsgType>(sub_topic, qos, cb);

  pubs_[pub_topic_if] = create_publisher<MsgType>(pub_topic_if, qos);
}

template <typename MsgType>
void ROSInterfaceNode::topicCallback(const typename MsgType::ConstSharedPtr& msg_in, const string& pub_topic)
{
  auto msg_out = std::make_unique<MsgType>(*msg_in);
  const auto pub = dynamic_pointer_cast<rclcpp::Publisher<MsgType>>(pubs_.at(pub_topic));
  pub->publish(move(msg_out));
}

string ROSInterfaceNode::throttled(const std::string& topic)
{
  return path::join(tobas::kThrottledTopicPrefix, topic);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ROSInterfaceNode)
