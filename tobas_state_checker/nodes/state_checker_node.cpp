#include <std_msgs/msg/bool.hpp>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_kdl_msgs_adapter/EulerStamped.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs/msg/event.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/action/land.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

using namespace std;

class StateCheckerNode : public tobas::BaseNode
{
  static constexpr double kWarnPeriod = 3.;                                // [s]
  static constexpr double kCpuTempertureThreshold = 80.;                   // [celsius]
  static constexpr double kAttitudeThreshold = 85. * tobas_std::kDeg2Rad;  // [rad]

  using self = StateCheckerNode;
  using super = tobas::BaseNode;

public:
  explicit StateCheckerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone::ConstSharedPtr drone_;
  std_msgs::msg::Bool::ConstSharedPtr arming_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::Event> event_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Cpu> cpu_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::EulerStamped> euler_sub_;

  // Service
  ros2::ServiceClientPtr<tobas_msgs::srv::SetArm> set_arm_sc_;

  void publishSystemCriticalEvent();
  void requestDisarmingRotors();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void eulerCb(const tobas_kdl_msgs::EulerStamped::ConstSharedPtr& euler);
};

StateCheckerNode::StateCheckerNode(const rclcpp::NodeOptions& options) : super("state_checker", options)
{
  event_pub_ = createPublisher<tobas_msgs::msg::Event>(tobas::kEventTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  cpu_sub_ = createSubscriber(tobas::kCpuTopic, &self::cpuCb, this);
  battery_sub_ = createSubscriber(tobas::kBatteryLpfTopic, &self::batteryCb, this);
  euler_sub_ = createSubscriber(tobas::kEulerTopic, &self::eulerCb, this);

  set_arm_sc_ = create_client<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv);
}

void StateCheckerNode::publishSystemCriticalEvent()
{
  auto event = std::make_unique<tobas_msgs::msg::Event>();
  event->data = tobas_msgs::msg::Event::SYSTEM_CRITICAL;
  event_pub_->publish(move(event));
}

void StateCheckerNode::requestDisarmingRotors()
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

void StateCheckerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
}

void StateCheckerNode::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void StateCheckerNode::cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu)
{
  if (arming_ == nullptr || !arming_->data)
    return;

  if (cpu->temperature > kCpuTempertureThreshold)
  {
    TOBAS_WARN_THROTTLE(
      kWarnPeriod, "CPU temperature is too high: ", cpu->temperature, " [C]. It is time to stop flying.");
  }
}

void StateCheckerNode::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  if (arming_ == nullptr || !arming_->data)
    return;

  if (drone_ == nullptr)
    return;

  if (battery->voltage < drone_->battery.sag_voltage)
  {
    TOBAS_WARN_THROTTLE(
      kWarnPeriod, "Battery voltage is too low: ", battery->voltage, " [V]. It is time to stop flying.");
  }
}

void StateCheckerNode::eulerCb(const tobas_kdl_msgs::EulerStamped::ConstSharedPtr& euler)
{
  if (arming_ == nullptr || !arming_->data)
    return;

  // 姿勢角が閾値を超えていたら全モータを非常停止
  // TODO: ここでパラシュートを開く
  if (max(abs(euler->euler.roll), abs(euler->euler.pitch)) > kAttitudeThreshold)
  {
    TOBAS_FATAL("The attitude angle exceeds the threshold. Stopping motors.");
    publishSystemCriticalEvent();
    requestDisarmingRotors();
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(StateCheckerNode)
