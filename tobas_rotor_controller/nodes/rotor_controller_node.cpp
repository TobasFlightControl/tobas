#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

using namespace std;

/* 推進系の目標推力を実現する． */
class RotorControllerNode : public tobas::BaseNode
{
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
    TOBAS_WARN_THROTTLE(
      tobas::kTypicalWarnPeriod, "Command is ignored because drone configuration has not been received yet.");
    return;
  }

  switch (drone_->prop->type())
  {
    case tobas::propulsion_system_t::ELECTRIC:
    {
      const auto eprop = dynamic_pointer_cast<tobas::ElectricPropulsionSystemConfig>(drone_->prop);

      // Create target speeds message
      auto tar_speeds_msg = std::make_unique<tobas_msgs::msg::RotorSpeedArray>();
      tar_speeds_msg->header = tar_thrusts_msg->header;

      // Convert target thrusts to target speeds
      for (const auto& tar_thrust_msg : tar_thrusts_msg->thrusts)
      {
        const auto& link_name = tar_thrust_msg.link_name;
        const auto& tar_thrust = tar_thrust_msg.thrust;
        const auto& rotor = eprop->getRotor(link_name);

        tar_speeds_msg->speeds.emplace_back();
        tar_speeds_msg->speeds.back().link_name = link_name;

        if (tar_thrust >= 0.)
        {
          tar_speeds_msg->speeds.back().speed = rotor->speedFromThrust(tar_thrust);
        }
        else
        {
          TOBAS_ERROR_THROTTLE(
            tobas::kTypicalErrorPeriod, "Negative thrust is commanded on rotor \"", link_name, "\": ", tar_thrust,
            " < 0 [N]");
          tar_speeds_msg->speeds.back().speed = 0.;
        }
      }

      // Publish target speeds
      tar_speeds_pub_->publish(move(tar_speeds_msg));

      break;
    }
    case tobas::propulsion_system_t::ICE:
    {
      const auto iprop = dynamic_pointer_cast<tobas::ElectricPropulsionSystemConfig>(drone_->prop);

      // TODO

      break;
    }
    default:
    {
      TOBAS_ERROR("Invalid propulsion system type: ", (int)drone_->prop->type());
      break;
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(RotorControllerNode)
