#include <std_msgs/msg/bool.hpp>

#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_kdl_msgs_adapter/EulerStamped.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs/msg/event.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

using namespace std;

class AttitudeCheckerNode : public tobas::BaseNode
{
  static constexpr double kAttitudeFatalThresh = tobas_std::deg2rad(85.);  // [rad]

  using self = AttitudeCheckerNode;
  using super = tobas::BaseNode;

public:
  explicit AttitudeCheckerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  std_msgs::msg::Bool::ConstSharedPtr arming_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::Event> event_pub_;

  // Subscribers
  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::EulerStamped> euler_sub_;

  // Service
  ros2::ServiceClientPtr<tobas_msgs::srv::SetArm> set_arm_sc_;

  void publishSystemCriticalEvent();
  void requestDisarmingRotors();

  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void eulerCb(const tobas_kdl_msgs::EulerStamped::ConstSharedPtr& euler);
};

AttitudeCheckerNode::AttitudeCheckerNode(const rclcpp::NodeOptions& options) : super("attitude_checker", options)
{
  event_pub_ = createPublisher<tobas_msgs::msg::Event>(tobas::kEventTopic);

  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  euler_sub_ = createSubscriber(path::join(tobas::kThrottledTopicNS, tobas::kEulerTopic), &self::eulerCb, this);

  set_arm_sc_ = create_client<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv);
}

void AttitudeCheckerNode::publishSystemCriticalEvent()
{
  auto event = std::make_unique<tobas_msgs::msg::Event>();
  event->data = tobas_msgs::msg::Event::SYSTEM_CRITICAL;
  event_pub_->publish(move(event));
}

void AttitudeCheckerNode::requestDisarmingRotors()
{
  if (!set_arm_sc_->service_is_ready())
  {
    TOBAS_ERROR("\"", tobas::kSetArmSrv, "\" service is not ready.");
    return;
  }

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = false;
  set_arm_sc_->async_send_request(req);
}

void AttitudeCheckerNode::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void AttitudeCheckerNode::eulerCb(const tobas_kdl_msgs::EulerStamped::ConstSharedPtr& euler)
{
  if (arming_ == nullptr || !arming_->data)
    return;

  // 姿勢角が閾値を超えていたら全モータを非常停止
  // TODO: ここでパラシュートを開く
  if (max(abs(euler->euler.roll), abs(euler->euler.pitch)) > kAttitudeFatalThresh)
  {
    TOBAS_FATAL("The attitude angle exceeds the threshold. Stopping motors.");
    publishSystemCriticalEvent();
    requestDisarmingRotors();
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(AttitudeCheckerNode)
