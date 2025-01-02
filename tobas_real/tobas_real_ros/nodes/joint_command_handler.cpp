#include <tobas_math/core.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

/**
 * @brief ジョイントの位置，速度，力のコマンドを受け取り，適切なハードウェアインターフェースに指令する．
 */
class JointCommandHandlerNode : public tobas::BaseNode
{
  using self = JointCommandHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit JointCommandHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  kdl::Tree tree_;
  tobas::Drone::ConstSharedPtr drone_;

  kdl::TreeJointParser joint_parser_;

  ros2::PublisherPtr<tobas_msgs::msg::PwmArray> pwms_pub_;

  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> positions_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> velocities_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> efforts_sub_;

  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions);
  void jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities);
  void jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts);
};

JointCommandHandlerNode::JointCommandHandlerNode(const rclcpp::NodeOptions& options)
  : super("real_joint_command_handler", options), joint_parser_(tree_)
{
  pwms_pub_ = createPublisher<tobas_msgs::msg::PwmArray>(tobas::kPwmCmdTopic);

  tree_sub_ = createSubscriber(tobas::kKDLTreeTopic, &self::treeCb, this, true, true);
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  positions_sub_ = createSubscriber(tobas::kJointPositionsCmdTopic, &self::jointPositionsCmdCb, this);
  velocities_sub_ = createSubscriber(tobas::kJointVelocitiesCmdTopic, &self::jointVelocitiesCmdCb, this);
  efforts_sub_ = createSubscriber(tobas::kJointEffortsCmdTopic, &self::jointEffortsCmdCb, this);
}

void JointCommandHandlerNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;
  joint_parser_.updateInternalDataStructures();
}

void JointCommandHandlerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
}

void JointCommandHandlerNode::jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions)
{
  if (tree_.getNrOfJoints() == 0)
    return;
  if (drone_ == nullptr)
    return;

  for (const auto& cmd : positions->commands)
  {
    const auto& jnt_name = cmd.name;

    // Get joint config
    const auto joint_it = drone_->joints.find(jnt_name);
    if (joint_it == drone_->joints.end())
    {
      TOBAS_WARN("Joint \"", jnt_name, "\" is not found.");
      return;
    }
    const auto& joint = joint_it->second;

    // Get limit
    const auto min_pos = joint_parser_.lowerLimit(joint.name);
    const auto max_pos = joint_parser_.upperLimit(joint.name);

    // Create real command messages
    auto pwms = std::make_unique<tobas_msgs::msg::PwmArray>();

    // Fill commands
    switch (joint.hw_iface)
    {
      case tobas::jnt_hw_iface_t::PWM:
        pwms->pwms.emplace_back();
        pwms->pwms.back().channel = joint.channel;
        pwms->pwms.back().period = math::remap<double>(cmd.data, min_pos, max_pos, tobas::kPwmMin, tobas::kPwmMax);
        break;
      case tobas::jnt_hw_iface_t::OTHER:
        break;
      default:
        TOBAS_WARN("The hardware interface of joint \"", jnt_name, "\" is invalid: ", (int)joint.hw_iface);
    }

    // Publish commands
    if (pwms->pwms.size() > 0)
    {
      pwms->header.stamp = positions->header.stamp;
      pwms_pub_->publish(std::move(pwms));
    }
  }
}

void JointCommandHandlerNode::jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities)
{
  if (drone_ == nullptr)
    return;

  (void)velocities;  // TODO
}

void JointCommandHandlerNode::jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts)
{
  if (drone_ == nullptr)
    return;

  (void)efforts;  // TODO
}

RCLCPP_COMPONENTS_REGISTER_NODE(JointCommandHandlerNode)
