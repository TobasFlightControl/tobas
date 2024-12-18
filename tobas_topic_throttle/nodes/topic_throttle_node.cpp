#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/rate_manager.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/sbus.hpp>
#include <tobas_msgs_adapter/imu_stamped.hpp>
#include <tobas_msgs_adapter/magnetic_field_stamped.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_kdl_msgs_adapter/euler_stamped.hpp>

using namespace std;

template <typename MsgType>
class TopicThrottle
{
  static constexpr double kPublishRate = 25.;  // [Hz]

public:
  explicit TopicThrottle() : rate_manager_(kPublishRate)
  {
  }

  void initialize(rclcpp::Node::SharedPtr node, const string& topic)
  {
    pub_ = ros2::createPublisher<MsgType>(node, path::join(tobas::kThrottledTopicNS, topic));
    sub_ = ros2::createSubscriber(node, topic, &TopicThrottle::callback, this);
  }

private:
  ros2::RateManager rate_manager_;

  ros2::PublisherPtr<MsgType> pub_;
  ros2::SubscriberPtr<MsgType> sub_;

  void callback(const typename MsgType::ConstSharedPtr& msg_in)
  {
    if (!rate_manager_.update(msg_in->header.stamp))
      return;

    auto msg_out = std::make_unique<MsgType>(*msg_in);
    pub_->publish(move(msg_out));
  }
};

class TopicThrottleNode : public tobas::BaseNode
{
  using self = TopicThrottleNode;
  using super = tobas::BaseNode;

public:
  explicit TopicThrottleNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void initialize();

private:
  TopicThrottle<tobas_msgs::msg::Battery> battery_throttle_;
  TopicThrottle<tobas_msgs::msg::RCInput> rcin_throttle_;
  TopicThrottle<tobas_msgs::msg::RotorStateArray> rotor_states_throttle_;
  TopicThrottle<tobas_msgs::Odometry> odom_throttle_;
  TopicThrottle<tobas_kdl_msgs::EulerStamped> euler_throttle_;
  TopicThrottle<tobas_msgs::ImuStamped> real_imu_throttle_;
  TopicThrottle<tobas_msgs::MagneticFieldStamped> real_mag_throttle_;
  TopicThrottle<tobas_msgs::msg::Sbus> real_sbus_throttle_;

  ros2::TimerPtr initialize_timer_;
};

TopicThrottleNode::TopicThrottleNode(const rclcpp::NodeOptions& options) : super("topic_throttle", options)
{
  initialize_timer_ = createTimer(0s, &self::initialize, this);
}

void TopicThrottleNode::initialize()
{
  const auto node = shared_from_this();

  battery_throttle_.initialize(node, tobas::kBatteryTopic);
  rcin_throttle_.initialize(node, tobas::kRcInputTopic);
  rotor_states_throttle_.initialize(node, tobas::kRotorStatesTopic);
  odom_throttle_.initialize(node, tobas::kOdometryTopic);
  euler_throttle_.initialize(node, tobas::kEulerTopic);
  real_imu_throttle_.initialize(node, real::kIMUTopic);
  real_mag_throttle_.initialize(node, real::kMagTopic);
  real_sbus_throttle_.initialize(node, real::kSBUSTopic);

  initialize_timer_->cancel();
}

RCLCPP_COMPONENTS_REGISTER_NODE(TopicThrottleNode)
