#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

using namespace std;

class RotorControllerNode : public tobas::BaseNode
{
  static constexpr double kCmdWarnPeriod = 1.;  // [s]

  using self = RotorControllerNode;
  using super = tobas::BaseNode;

public:
  explicit RotorControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone::ConstSharedPtr drone_;

  ros2::PublisherPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_pub_;
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_sub_;

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void thrustsCmdCb(const tobas_msgs::msg::RotorThrustArray::ConstSharedPtr& tar_thrusts_msg);
};

RotorControllerNode::RotorControllerNode(const rclcpp::NodeOptions& options) : super("rotor_controller", options)
{
  tar_speeds_pub_ = createPublisher<tobas_msgs::msg::RotorSpeedArray>(tobas::kRotorSpeedsCmdTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tar_thrusts_sub_ = createSubscriber(tobas::kRotorThrustsCmdTopic, &self::thrustsCmdCb, this);
}

void RotorControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
}

void RotorControllerNode::thrustsCmdCb(const tobas_msgs::msg::RotorThrustArray::ConstSharedPtr& tar_thrusts_msg)
{
  if (drone_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(kCmdWarnPeriod, "Command is ignored because drone configuration has not been received yet.");
    return;
  }

  // Create target speeds message
  auto tar_speeds_msg = std::make_unique<tobas_msgs::msg::RotorSpeedArray>();
  tar_speeds_msg->header = tar_thrusts_msg->header;

  // Convert target thrusts to target speeds
  for (const auto& tar_thrust_msg : tar_thrusts_msg->thrusts)
  {
    const auto& channel = tar_thrust_msg.channel;
    const auto& tar_thrust = tar_thrust_msg.thrust;

    tar_speeds_msg->speeds.emplace_back();
    tar_speeds_msg->speeds.back().channel = channel;

    if (tar_thrust >= 0.)
    {
      tar_speeds_msg->speeds.back().speed = drone_->rotSpeedFromThrust(channel, tar_thrust_msg.thrust);
    }
    else
    {
      TOBAS_WARN_THROTTLE(kCmdWarnPeriod, "Negative thrust is commanded on CH", channel, ": ", tar_thrust, " < 0 [N]");
      tar_speeds_msg->speeds.back().speed = 0.;
    }
  }

  // Publish target speeds
  tar_speeds_pub_->publish(move(tar_speeds_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(RotorControllerNode)
