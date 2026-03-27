#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>

namespace tobas
{
namespace real
{
/**
 * @brief ジョイントの位置，速度，力のコマンドを受け取り，適切なハードウェアインターフェースに指令する．
 * また，そのジョイントの状態を発行する．
 */
class JointsHandlerNode : public BaseNode
{
  using self = JointsHandlerNode;
  using super = BaseNode;

public:
  explicit JointsHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  Drone::ConstSharedPtr drone_;

  bool pos_commanded_ = false;
  bool vel_commanded_ = false;
  bool eff_commanded_ = false;

  ros2::PublisherPtr<tobas_msgs::msg::PwmArray> pwms_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::JointStateArray> joint_states_pub_;

  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> positions_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> velocities_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> efforts_sub_;

  ros2::TimerPtr pos_reset_timer_;
  ros2::TimerPtr vel_reset_timer_;
  ros2::TimerPtr eff_reset_timer_;

  double pwmPeriodFromJointPos(const PwmConfig& pwm, double cmd_pos);

  void droneCb(const Drone::ConstSharedPtr& drone);
  void jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions);
  void jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities);
  void jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts);

  void positionResetTimerCb();
  void velocityResetTimerCb();
  void effortResetTimerCb();
};

JointsHandlerNode::JointsHandlerNode(const rclcpp::NodeOptions& options)
  : super("real_joints_handler", nodeOptions_Default(options))
{
  pwms_pub_ = createPublisher<tobas_msgs::msg::PwmArray>(topic::kPwmCmd);
  joint_states_pub_ = createPublisher<tobas_msgs::msg::JointStateArray>(topic::kJointStates);

  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
  positions_sub_ = createSubscriber(topic::kJointPosCmd, &self::jointPositionsCmdCb, this);
  velocities_sub_ = createSubscriber(topic::kJointVelCmd, &self::jointVelocitiesCmdCb, this);
  efforts_sub_ = createSubscriber(topic::kJointEffCmd, &self::jointEffortsCmdCb, this);

  pos_reset_timer_ = createWallTimer(kCommandAutoResetTimeout, &self::positionResetTimerCb, this, false);
  vel_reset_timer_ = createWallTimer(kCommandAutoResetTimeout, &self::velocityResetTimerCb, this, false);
  eff_reset_timer_ = createWallTimer(kCommandAutoResetTimeout, &self::effortResetTimerCb, this, false);
}

double JointsHandlerNode::pwmPeriodFromJointPos(const PwmConfig& pwm, double cmd_pos)
{
  cmd_pos = std::clamp(cmd_pos, pwm.value_range.first, pwm.value_range.second);
  return pwm.periodFromValue(cmd_pos);
}

void JointsHandlerNode::droneCb(const Drone::ConstSharedPtr& drone)
{
  drone_ = drone;

  // オートリセットタイマーを起動または停止
  bool has_pos = false;
  bool has_vel = false;
  bool has_eff = false;

  for (const auto& [_, joint] : drone_->joints) {
    switch (joint.cmd_iface) {
      case JointCommandInterface::kPosition:
        has_pos = true;
        break;
      case JointCommandInterface::kVelocity:
        has_vel = true;
        break;
      case JointCommandInterface::kEffort:
        has_eff = true;
        break;
      case JointCommandInterface::kNone:
        break;
      default:
        TOBAS_ERROR("Invalid joint command interface.");
        break;
    }
  }

  if (has_pos) {
    pos_reset_timer_->reset();
  }
  else {
    pos_reset_timer_->cancel();
  }

  if (has_vel) {
    vel_reset_timer_->reset();
  }
  else {
    vel_reset_timer_->cancel();
  }

  if (has_eff) {
    eff_reset_timer_->reset();
  }
  else {
    eff_reset_timer_->cancel();
  }
}

void JointsHandlerNode::jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions)
{
  if (!drone_) {
    return;
  }

  // Create messages
  auto pwms = std::make_unique<tobas_msgs::msg::PwmArray>();
  auto joint_states = std::make_unique<tobas_msgs::msg::JointStateArray>();

  for (const auto& cmd : positions->commands) {
    const auto& jnt_name = cmd.name;
    auto cmd_pos = cmd.data;

    // Get joint config
    const auto joint_it = drone_->joints.find(jnt_name);
    if (joint_it == drone_->joints.end()) {
      TOBAS_ERROR("Joint \"", jnt_name, "\" is not found.");
      continue;
    }
    const auto& joint = joint_it->second;

    // Fill commands
    switch (joint.hw_iface) {
      case HardwareInterface::kPwm: {
        const auto& pwm_cfg = drone_->pwms.at(joint.name);

        pwms->pwms.emplace_back();
        pwms->pwms.back().channel = pwm_cfg.channel;
        pwms->pwms.back().period = pwmPeriodFromJointPos(pwm_cfg, cmd_pos);

        joint_states->states.emplace_back();
        joint_states->states.back().name = joint.name;
        joint_states->states.back().position = cmd_pos;
        joint_states->states.back().velocity = NAN;
        joint_states->states.back().effort = NAN;

        break;
      }
      case HardwareInterface::kOther: {
        break;
      }
      default: {
        TOBAS_ERROR("The hardware interface of joint \"", jnt_name, "\" is invalid: ", (int)joint.hw_iface);
        break;
      }
    }
  }

  // Publish messages
  if (!pwms->pwms.empty()) {
    pwms->header.stamp = positions->header.stamp;
    pwms_pub_->publish(std::move(pwms));
  }
  if (!joint_states->states.empty()) {
    joint_states->header.stamp = positions->header.stamp;
    joint_states_pub_->publish(std::move(joint_states));
  }

  // Reset timeout timer
  pos_commanded_ = true;
  pos_reset_timer_->reset();
}

void JointsHandlerNode::jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities)
{
  if (!drone_) {
    return;
  }

  (void)velocities;  // TODO

  // Reset timeout timer
  vel_commanded_ = true;
  vel_reset_timer_->reset();
}

void JointsHandlerNode::jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts)
{
  if (!drone_) {
    return;
  }

  (void)efforts;  // TODO

  // Reset timeout timer
  eff_commanded_ = true;
  eff_reset_timer_->reset();
}

void JointsHandlerNode::positionResetTimerCb()
{
  // Create messages
  auto pwms = std::make_unique<tobas_msgs::msg::PwmArray>();
  auto joint_states = std::make_unique<tobas_msgs::msg::JointStateArray>();

  for (const auto& [_, joint] : drone_->joints) {
    if (!joint.isServoJoint()) {
      continue;
    }

    if (joint.cmd_iface != JointCommandInterface::kPosition) {
      continue;
    }

    // Fill commands
    switch (joint.hw_iface) {
      case HardwareInterface::kPwm: {
        const auto& pwm_cfg = drone_->pwms.at(joint.name);

        pwms->pwms.emplace_back();
        pwms->pwms.back().channel = pwm_cfg.channel;
        pwms->pwms.back().period = pwm_cfg.periodFromValue(joint.home_pos);

        joint_states->states.emplace_back();
        joint_states->states.back().name = joint.name;
        joint_states->states.back().position = joint.home_pos;
        joint_states->states.back().velocity = NAN;
        joint_states->states.back().effort = NAN;

        break;
      }
      case HardwareInterface::kOther: {
        break;
      }
      default: {
        TOBAS_WARN("The hardware interface of joint \"", joint.name, "\" is invalid: ", (int)joint.hw_iface);
        break;
      }
    }
  }

  // Publish messages
  if (!pwms->pwms.empty()) {
    pwms->header.stamp = now();
    pwms_pub_->publish(std::move(pwms));
  }
  if (!joint_states->states.empty()) {
    joint_states->header.stamp = now();
    joint_states_pub_->publish(std::move(joint_states));
  }

  // Warn if commanded positions are reset
  if (pos_commanded_) {
    pos_commanded_ = false;
    TOBAS_WARN(
      "All joints with position command interface are reset to home position because ",
      kCommandAutoResetTimeout,
      " have elapsed since the last command.");
  }
}

void JointsHandlerNode::velocityResetTimerCb()
{
  // TODO
}

void JointsHandlerNode::effortResetTimerCb()
{
  // TODO
}
}  // namespace real
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::real::JointsHandlerNode)
