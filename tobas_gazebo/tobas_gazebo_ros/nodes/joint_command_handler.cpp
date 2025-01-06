#include <std_msgs/msg/float64_multi_array.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

using namespace std;

/**
 * @brief ジョイントの位置，速度，力のコマンドを受け取り，Gazeboのトランスミッションに指令する．
 */
class JointCommandHandlerNode : public tobas::BaseNode
{
  using self = JointCommandHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit JointCommandHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone::ConstSharedPtr drone_;

  unordered_map<string, pair<tobas::jnt_cmd_iface_t, ros2::PublisherPtr<std_msgs::msg::Float64MultiArray>>> ctrl_map_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> positions_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> velocities_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> efforts_sub_;

  ros2::ServiceClientPtr<controller_manager_msgs::srv::ListControllers> list_controllers_sc_;

  ros2::TimerPtr pos_reset_timer_;
  ros2::TimerPtr vel_reset_timer_;
  ros2::TimerPtr eff_reset_timer_;

  void publishJointCommand(const std::string& jnt_name, double command);
  void publishJointCommand(const tobas_msgs::msg::JointCommand& cmd);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions);
  void jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities);
  void jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts);

  void positionResetTimerCb();
  void velocityResetTimerCb();
  void effortResetTimerCb();
};

JointCommandHandlerNode::JointCommandHandlerNode(const rclcpp::NodeOptions& options)
  : super("gazebo_joint_command_handler", options)
{
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this);
}

void JointCommandHandlerNode::publishJointCommand(const std::string& jnt_name, double command)
{
  auto gz_cmd = std::make_unique<std_msgs::msg::Float64MultiArray>();
  gz_cmd->data.push_back(command);

  const auto& publisher = ctrl_map_.at(jnt_name).second;
  publisher->publish(move(gz_cmd));
}

void JointCommandHandlerNode::publishJointCommand(const tobas_msgs::msg::JointCommand& cmd)
{
  publishJointCommand(cmd.name, cmd.data);
}

void JointCommandHandlerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;

  // Resister publishers
  ctrl_map_.clear();
  for (const auto& [_, joint] : drone->joints)
  {
    const auto controller_name = joint.name + "_controller";
    const auto topic = controller_name + "/commands";
    ctrl_map_[joint.name] = { static_cast<tobas::jnt_cmd_iface_t>(joint.cmd_iface),
                              createPublisher<std_msgs::msg::Float64MultiArray>(topic, false, true) };
  }

  // Resister subscribers
  positions_sub_ = createSubscriber(tobas::kJointPositionsCmdTopic, &self::jointPositionsCmdCb, this);
  velocities_sub_ = createSubscriber(tobas::kJointVelocitiesCmdTopic, &self::jointVelocitiesCmdCb, this);
  efforts_sub_ = createSubscriber(tobas::kJointEffortsCmdTopic, &self::jointEffortsCmdCb, this);

  // Resister timers
  pos_reset_timer_ = createTimer(tobas::kCommandAutoResetTimeout, &self::positionResetTimerCb, this);
  vel_reset_timer_ = createTimer(tobas::kCommandAutoResetTimeout, &self::velocityResetTimerCb, this);
  eff_reset_timer_ = createTimer(tobas::kCommandAutoResetTimeout, &self::effortResetTimerCb, this);
}

void JointCommandHandlerNode::jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions)
{
  for (const auto& tbs_cmd : positions->commands)
  {
    const auto& jnt_name = tbs_cmd.name;
    if (!ctrl_map_.contains(jnt_name))
    {
      TOBAS_ERROR("Controller for joint \"", jnt_name, "\" is not found.");
      continue;
    }

    const auto& cmd_iface = ctrl_map_[jnt_name].first;
    if (cmd_iface != tobas::jnt_cmd_iface_t::POSITION)
    {
      TOBAS_ERROR(
        "The command interface of joint \"", jnt_name, "\" is not position. So received position command for joint \"",
        jnt_name, "\" is ignored.");
      continue;
    }

    publishJointCommand(tbs_cmd);
  }

  pos_reset_timer_->reset();
}

void JointCommandHandlerNode::jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities)
{
  for (const auto& tbs_cmd : velocities->commands)
  {
    const auto& jnt_name = tbs_cmd.name;

    if (!ctrl_map_.contains(jnt_name))
    {
      TOBAS_ERROR("Controller for joint \"", jnt_name, "\" is not found.");
      continue;
    }

    const auto& cmd_iface = ctrl_map_[jnt_name].first;
    if (cmd_iface != tobas::jnt_cmd_iface_t::VELOCITY)
    {
      TOBAS_ERROR(
        "The command interface of joint \"", jnt_name, "\" is not velocity. So received velocity command for joint \"",
        jnt_name, "\" is ignored.");
      continue;
    }

    publishJointCommand(tbs_cmd);
  }

  vel_reset_timer_->reset();
}

void JointCommandHandlerNode::jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts)
{
  for (const auto& tbs_cmd : efforts->commands)
  {
    const auto& jnt_name = tbs_cmd.name;
    if (!ctrl_map_.contains(jnt_name))
    {
      TOBAS_ERROR("Controller for joint \"", jnt_name, "\" is not found.");
      continue;
    }

    const auto& cmd_iface = ctrl_map_[jnt_name].first;
    if (cmd_iface != tobas::jnt_cmd_iface_t::EFFORT)
    {
      TOBAS_ERROR(
        "The command interface of joint \"", jnt_name, "\" is not effort. So received effort command for joint \"",
        jnt_name, "\" is ignored.");
      continue;
    }

    publishJointCommand(tbs_cmd);
  }

  eff_reset_timer_->reset();
}

void JointCommandHandlerNode::positionResetTimerCb()
{
  for (const auto& [_, joint] : drone_->joints)
  {
    if (!joint.isServoJoint())
      continue;
    if (joint.cmd_iface != tobas::jnt_cmd_iface_t::POSITION)
      continue;

    publishJointCommand(joint.name, joint.home_pos);
  }
}

void JointCommandHandlerNode::velocityResetTimerCb()
{
  for (const auto& [_, joint] : drone_->joints)
  {
    if (!joint.isServoJoint())
      continue;
    if (joint.cmd_iface != tobas::jnt_cmd_iface_t::VELOCITY)
      continue;

    publishJointCommand(joint.name, joint.home_pos);
  }
}

void JointCommandHandlerNode::effortResetTimerCb()
{
  for (const auto& [_, joint] : drone_->joints)
  {
    if (!joint.isServoJoint())
      continue;
    if (joint.cmd_iface != tobas::jnt_cmd_iface_t::EFFORT)
      continue;

    publishJointCommand(joint.name, joint.home_pos);
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(JointCommandHandlerNode)
