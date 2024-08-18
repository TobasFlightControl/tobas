#include <std_srvs/srv/trigger.hpp>

#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_msgs/Drone.hpp>
#include <tobas_msgs/msg/throttle_array.hpp>
#include <tobas_msgs/srv/enable_rc_output.hpp>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>

using namespace std;

class RotorCommandHandlerNode : public tobas::BaseNode
{
  using self = RotorCommandHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit RotorCommandHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  std::map<uint8_t, PublisherPtr<tobas_gazebo_msgs::msg::Throttle>> throttle_pubs_;
  SubscriberPtr<tobas::Drone> drone_sub_;
  SubscriberPtr<tobas_msgs::msg::ThrottleArray> throttles_sub_;
  ServicePtr<tobas_msgs::srv::EnableRCOutput> enable_rcout_srv_;

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void throttlesCb(const tobas_msgs::msg::ThrottleArray::ConstSharedPtr& throttles);
  void enableRCOutputCb(
    const tobas_msgs::srv::EnableRCOutput::Request::ConstSharedPtr& req,
    const tobas_msgs::srv::EnableRCOutput::Response::SharedPtr& res);
};

RotorCommandHandlerNode::RotorCommandHandlerNode(const rclcpp::NodeOptions& options)
  : super("gazebo_rotor_command_handler", options)
{
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true);
}

void RotorCommandHandlerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  throttle_pubs_.clear();
  for (const auto& rotor : drone->rotors)
  {
    const auto topic = string(gazebo::kThrottleTopicPrefix) + "_" + to_string(rotor.channel);
    throttle_pubs_[rotor.channel] = createPublisher<tobas_gazebo_msgs::msg::Throttle>(topic);
  }

  throttles_sub_ = createSubscriber(tobas::kThrottlesCmdTopic, &self::throttlesCb, this);
  enable_rcout_srv_ =
    createService<tobas_msgs::srv::EnableRCOutput>(tobas::kEnableRcOutputSrv, &self::enableRCOutputCb, this);

  TOBAS_INFO("Gazego rotor command handler is initialized.");
}

void RotorCommandHandlerNode::throttlesCb(const tobas_msgs::msg::ThrottleArray::ConstSharedPtr& throttles)
{
  for (const auto& in : throttles->throttles)
  {
    // Check channel
    if (!throttle_pubs_.contains(in.channel))
    {
      TOBAS_ERROR("The drone does not have rotor channel ", in.channel, ".");
      return;
    }

    // Create throttle message
    auto out = std::make_unique<tobas_gazebo_msgs::msg::Throttle>();
    out->header = throttles->header;
    out->data = in.throttle;

    // Publish throttle message
    throttle_pubs_.at(in.channel)->publish(move(out));
  }
}

void RotorCommandHandlerNode::enableRCOutputCb(
  const tobas_msgs::srv::EnableRCOutput::Request::ConstSharedPtr& req,
  const tobas_msgs::srv::EnableRCOutput::Response::SharedPtr& res)
{
  if (!throttle_pubs_.contains(req->channel))
  {
    res->success = false;
    res->message = "The drone does not have rotor channel " + to_string(req->channel) + ".";
    return;
  }

  // TODO: ちゃんとサービスを実装する

  res->success = true;
  res->message = "";
  return;
}

RCLCPP_COMPONENTS_REGISTER_NODE(RotorCommandHandlerNode)
