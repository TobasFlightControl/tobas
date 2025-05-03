#include <tobas_kdl/tree_active_joints_extractor.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/tree_joint_state_converter.hpp>

#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs_adapter/link_state_array.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

#include "../include/tobas_manipulation/constants.hpp"

using namespace std;

class PositionControllerNode : public tobas::BaseNode
{
  using self = PositionControllerNode;
  using super = tobas::BaseNode;

public:
  explicit PositionControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone::ConstSharedPtr drone_;

  tobas_msgs::msg::JointStateArray home_js_;

  tobas_msgs::msg::JointStateArray::ConstSharedPtr tar_js_;
  tobas_msgs::LinkStateArray::ConstSharedPtr tar_ls_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> positions_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> cur_js_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> tar_js_sub_;
  ros2::SubscriberPtr<tobas_msgs::LinkStateArray> tar_ls_sub_;

  // Timer
  ros2::TimerPtr auto_reset_timer_;

  bool jointSpaceControl(tobas_msgs::msg::JointCommandArray& positions_msg);
  bool taskSpaceControl(tobas_msgs::msg::JointCommandArray& positions_msg);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void currentJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& cur_js);
  void targetJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& tar_js);
  void targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls);

  void autoResetTimerCb();
};

PositionControllerNode::PositionControllerNode(const rclcpp::NodeOptions& options)
  : super("position_controller", options)
{
  positions_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(tobas::kJointPosCmdTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  cur_js_sub_ = createSubscriber(tobas::kJointStatesTopic, &self::currentJointStateCb, this);
  tar_js_sub_ = createSubscriber(tobas::kPosCtrlJSTopic, &self::targetJointStateCb, this);
  tar_ls_sub_ = createSubscriber(tobas::kPosCtrlLSTopic, &self::targetLinkStateCb, this);

  auto_reset_timer_ = createTimer(manipulation::kAutoResetTimeThresh, &self::autoResetTimerCb, this, false);
}

bool PositionControllerNode::jointSpaceControl(tobas_msgs::msg::JointCommandArray& positions_msg)
{
  // 位置コマンドをそのまま流すだけ
  for (const auto& cmd : tar_js_->states) {
    const auto& joint = drone_->joints.at(cmd.name);
    if (joint.role != tobas::jnt_role_t::MANIPULATION) {
      TOBAS_WARN("The role of joint \"", cmd.name, "\" must be \"MANIPULATION\".");
      continue;
    }
    if (joint.cmd_iface != tobas::jnt_cmd_iface_t::POSITION) {
      TOBAS_WARN("The command interface of joint \"", cmd.name, "\" must be \"POSITION\".");
      continue;
    }

    positions_msg.commands.emplace_back();
    positions_msg.commands.back().name = cmd.name;
    positions_msg.commands.back().data = cmd.position;
  }

  return true;
}

bool PositionControllerNode::taskSpaceControl(tobas_msgs::msg::JointCommandArray&)
{
  TOBAS_ERROR("Task space control of joint position controller is not implemented.");  // TODO

  return true;
}

void PositionControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;

  home_js_.states.clear();

  // 位置指令タイプの関節のホームポジションを取得
  for (const auto& [jnt_name, jnt_cfg] : drone->joints) {
    if (jnt_cfg.role != tobas::jnt_role_t::MANIPULATION) {
      continue;
    }
    if (jnt_cfg.cmd_iface != tobas::jnt_cmd_iface_t::POSITION) {
      continue;
    }
    home_js_.states.emplace_back();
    home_js_.states.back().name = jnt_name;
    home_js_.states.back().position = jnt_cfg.home_pos;
  }

  // ホームポジションを初期目標状態に設定
  if (home_js_.states.size() > 0) {
    tar_js_ = std::make_shared<tobas_msgs::msg::JointStateArray>(home_js_);
  }
}

void PositionControllerNode::currentJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr&)
{
  if (home_js_.states.size() == 0) {
    return;
  }
  if (!tar_js_ && !tar_ls_) {
    return;
  }

  // Create joint velocities command
  auto positions_msg = std::make_unique<tobas_msgs::msg::JointCommandArray>();

  // Joint space control or Task space control
  if (tar_js_) {
    if (!jointSpaceControl(*positions_msg)) {
      return;
    }
  }
  else if (tar_ls_) {
    if (!taskSpaceControl(*positions_msg)) {
      return;
    }
  }
  else {
    TOBAS_ERROR("Both target joint state and target link state are NULL.");
    return;
  }

  // Publish joint velocities command
  positions_pub_->publish(move(positions_msg));
}

void PositionControllerNode::targetJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& tar_js)
{
  tar_js_ = tar_js;
  tar_ls_.reset();

  auto_reset_timer_->reset();
}

void PositionControllerNode::targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls)
{
  tar_ls_ = tar_ls;
  tar_js_.reset();

  auto_reset_timer_->reset();
}

void PositionControllerNode::autoResetTimerCb()
{
  tar_js_ = std::make_shared<tobas_msgs::msg::JointStateArray>(home_js_);
  tar_ls_.reset();

  TOBAS_WARN(
    "The target joint states are automatically reset because ",
    manipulation::kAutoResetTimeThresh,
    " have elapsed since the last command.");

  auto_reset_timer_->cancel();
}

RCLCPP_COMPONENTS_REGISTER_NODE(PositionControllerNode)
