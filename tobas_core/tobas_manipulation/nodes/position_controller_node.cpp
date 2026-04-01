// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_kdl/tree_active_joints_extractor.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_node/node.hpp>
#include <tobas_tools/tree_joint_state_converter.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs_adapter/link_state_array.hpp>

#include "tobas_manipulation/constants.hpp"

namespace tobas
{
namespace manipulation
{
class PositionControllerNode : public BaseNode
{
  using self = PositionControllerNode;
  using super = BaseNode;

public:
  explicit PositionControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Parameters
  std::unordered_set<std::string> jnt_names_;

  Drone::ConstSharedPtr drone_;

  tobas_msgs::msg::JointStateArray home_js_;

  tobas_msgs::msg::JointStateArray::ConstSharedPtr tar_js_;
  tobas_msgs::LinkStateArray::ConstSharedPtr tar_ls_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> positions_pub_;

  // Subscribers
  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> cur_js_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> tar_js_sub_;
  ros2::SubscriberPtr<tobas_msgs::LinkStateArray> tar_ls_sub_;

  // Timer
  ros2::TimerPtr auto_reset_timer_;

  bool jointSpaceControl(tobas_msgs::msg::JointCommandArray& positions_msg);
  bool taskSpaceControl(tobas_msgs::msg::JointCommandArray& positions_msg);

  void droneCb(const Drone::ConstSharedPtr& drone);
  void currentJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& cur_js);
  void targetJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& tar_js);
  void targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls);

  void autoResetTimerCb();
};

PositionControllerNode::PositionControllerNode(const rclcpp::NodeOptions& options)
  : super("jointpos_trajectory_controller", nodeOptions_Default(options))
{
  const auto jnt_names = getStringArrayParam("joint_names", {});
  if (jnt_names.empty()) {
    TOBAS_ERROR("Joint names are not specified.");
    return;
  }
  jnt_names_.insert(jnt_names.begin(), jnt_names.end());

  positions_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(topic::kJointPosCmd);

  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
  cur_js_sub_ = createSubscriber(topic::kJointStates, &self::currentJointStateCb, this);
  tar_js_sub_ = createSubscriber(topic::kPosCtrlJS, &self::targetJointStateCb, this);
  tar_ls_sub_ = createSubscriber(topic::kPosCtrlLS, &self::targetLinkStateCb, this);

  auto_reset_timer_ = createTimer(manipulation::kAutoResetTimeThresh, &self::autoResetTimerCb, this, false);
}

bool PositionControllerNode::jointSpaceControl(tobas_msgs::msg::JointCommandArray& positions_msg)
{
  // 位置コマンドをそのまま流すだけ
  for (const auto& tar_state : tar_js_->states) {
    const auto& jnt_name = tar_state.name;
    if (!jnt_names_.contains(jnt_name)) {
      TOBAS_ERROR("The target joint \"", jnt_name, "\" is not included in the joint group.");
      return false;
    }
    positions_msg.commands.emplace_back();
    positions_msg.commands.back().name = jnt_name;
    positions_msg.commands.back().data = tar_state.position;
  }

  return true;
}

bool PositionControllerNode::taskSpaceControl(tobas_msgs::msg::JointCommandArray&)
{
  TOBAS_ERROR("Task space control of joint position controller is not implemented.");  // TODO

  return true;
}

void PositionControllerNode::droneCb(const Drone::ConstSharedPtr& drone)
{
  drone_ = drone;

  home_js_.states.clear();

  // ジョイントのホームポジションを取得
  for (const auto& jnt_name : jnt_names_) {
    const auto joint_it = drone->joints.find(jnt_name);
    if (joint_it == drone->joints.end()) {
      TOBAS_WARN("The drone does not have joint \"", jnt_name, "\".");
      continue;
    }
    const auto& joint = joint_it->second;
    if (joint.cmd_iface != JointCommandInterface::kPosition) {
      TOBAS_WARN("The command interface of joint \"", jnt_name, "\" is not position.");
      continue;
    }
    home_js_.states.emplace_back();
    home_js_.states.back().name = jnt_name;
    home_js_.states.back().position = joint.home_pos;
  }

  // ホームポジションを初期目標状態に設定
  if (!home_js_.states.empty()) {
    tar_js_ = std::make_shared<tobas_msgs::msg::JointStateArray>(home_js_);
  }
}

void PositionControllerNode::currentJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr&)
{
  if (home_js_.states.empty()) {
    return;
  }
  if (!tar_js_ && !tar_ls_) {
    return;
  }

  // Create joint positions command
  auto positions_msg = std::make_unique<tobas_msgs::msg::JointCommandArray>();
  positions_msg->header.stamp = now();

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
    TOBAS_ERROR("Both target joint state and target link state are null.");
    return;
  }

  // Publish joint positions command
  positions_pub_->publish(std::move(positions_msg));
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
}  // namespace manipulation
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::manipulation::PositionControllerNode)
