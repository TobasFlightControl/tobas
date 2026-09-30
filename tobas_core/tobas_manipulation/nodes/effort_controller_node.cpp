// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <optional>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_kdl/tree_jntspace_pid.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_kdl/tree_taskspace_pid.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/tf_listener.hpp>
#include <tobas_tools/tree_joint_state_converter.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs_adapter/link_state_array.hpp>

#include "tobas_manipulation/constants.hpp"
#include "tobas_manipulation/util.hpp"

using namespace std::chrono_literals;

namespace tobas
{
namespace manipulation
{
class EffortControllerNode : public BaseNode
{
  using self = EffortControllerNode;
  using super = BaseNode;

public:
  explicit EffortControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Parameters
  std::unordered_set<std::string> jnt_names_;

  Drone::ConstSharedPtr drone_;
  kdl::Tree tree_;

  kdl::TreeJointParser jnt_parser_;
  kdl::TreeJntSpacePID pid_js_;
  kdl::TreeTaskSpacePID pid_ts_;
  TreeJointStateConverter cur_js_conv_;
  TreeJointStateConverter tar_js_conv_;

  std::optional<ros2::TransformListener> tf_listener_;
  tobas_msgs::msg::JointStateArray home_js_;

  tobas_msgs::msg::JointStateArray::ConstSharedPtr tar_js_;
  tobas_msgs::LinkStateArray::ConstSharedPtr tar_ls_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> efforts_pub_;

  // Subscribers
  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> cur_js_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> tar_js_sub_;
  ros2::SubscriberPtr<tobas_msgs::LinkStateArray> tar_ls_sub_;

  // Timer
  ros2::TimerPtr initialize_timer_;
  ros2::TimerPtr auto_reset_timer_;

  void initialize();

  bool jointSpaceControl(
    const tobas_msgs::msg::JointStateArray& cur_js,
    const tobas_msgs::msg::JointStateArray& tar_js,
    tobas_msgs::msg::JointCommandArray& efforts_msg);
  bool taskSpaceControl(
    const tobas_msgs::msg::JointStateArray& cur_js,
    const tobas_msgs::LinkStateArray& tar_ls,
    tobas_msgs::msg::JointCommandArray& efforts_msg);

  void jointStiffnessCb(const long& p);
  void jointDamping(const long& p);
  void linearStiffnessCb(const long& p);
  void angularStiffnessCb(const long& p);
  void linearDampingCb(const long& p);
  void angularDampingCb(const long& p);

  void droneCb(const Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void currentJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& cur_js);
  void targetJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& tar_js);
  void targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls);

  void autoResetTimerCb();
};

EffortControllerNode::EffortControllerNode(const rclcpp::NodeOptions& options)
  : super("jointeff_trajectory_controller", nodeOptions_DParam(options))
  , jnt_parser_(tree_)
  , pid_js_(tree_)
  , pid_ts_(tree_)
  , cur_js_conv_(tree_)
  , tar_js_conv_(tree_)
{
  initialize_timer_ = createTimer(0s, &self::initialize, this);
}

void EffortControllerNode::initialize()
{
  const auto jnt_names = getStringArrayParam("joint_names", {});
  if (jnt_names.empty()) {
    TOBAS_ERROR("Joint names are not specified.");
    return;
  }
  jnt_names_.insert(jnt_names.begin(), jnt_names.end());

  // `shared_from_this` cannot be called from the constructor.
  tf_listener_.emplace(shared_from_this());

  addDynamicIntParam("joint_stiffness", &self::jointStiffnessCb, this, 5, 5, 1, 20);
  addDynamicIntParam("joint_damping", &self::jointDamping, this, 1, 10, 1, 20);
  addDynamicIntParam("linear_stiffness", &self::linearStiffnessCb, this, 5, 5, 1, 20);
  addDynamicIntParam("angular_stiffness", &self::angularStiffnessCb, this, 5, 5, 1, 20);
  addDynamicIntParam("linear_damping", &self::linearDampingCb, this, 1, 10, 1, 20);
  addDynamicIntParam("angular_damping", &self::angularDampingCb, this, 1, 10, 1, 20);

  efforts_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(topic::kJointEffCmd);

  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(topic::kKdlTree, &self::treeCb, this, true, true);
  cur_js_sub_ = createSubscriber(topic::kJointStates, &self::currentJointStateCb, this);
  tar_js_sub_ = createSubscriber(topic::kEffCtrlJS, &self::targetJointStateCb, this);
  tar_ls_sub_ = createSubscriber(topic::kEffCtrlLS, &self::targetLinkStateCb, this);

  auto_reset_timer_ = createTimer(kAutoResetTimeThresh, &self::autoResetTimerCb, this, false);

  initialize_timer_->cancel();
}

bool EffortControllerNode::jointSpaceControl(
  const tobas_msgs::msg::JointStateArray& cur_js,
  const tobas_msgs::msg::JointStateArray& tar_js,
  tobas_msgs::msg::JointCommandArray& efforts_msg)
{
  // JointState -> JntArray
  if (const auto result = cur_js_conv_.convert(cur_js); !result) {
    TOBAS_ERROR("Failed to convert current JointState to Jntarray: ", result.error());
    return false;
  }
  if (const auto result = tar_js_conv_.convert(tar_js); !result) {
    TOBAS_ERROR("Failed to convert target JointState to Jntarray: ", result.error());
    return false;
  }

  const auto& cur_q = cur_js_conv_.getPosition();
  const auto& cur_qd = cur_js_conv_.getVelocity();
  const auto& tar_q = tar_js_conv_.getPosition();
  const auto& tar_qd = tar_js_conv_.getVelocity();

  // Calculate joint torques with PID.
  const auto& efforts_ff = tar_js_conv_.getEffort();
  const auto& efforts_fb = pid_js_.cartToJnt(cur_q, cur_qd, tar_q, tar_qd);
  const auto efforts = efforts_ff + efforts_fb;

  // Fill output message.
  for (const auto& tar_state : tar_js.states) {
    const auto& jnt_name = tar_state.name;
    if (!jnt_names_.contains(jnt_name)) {
      TOBAS_ERROR("The target joint '", jnt_name, "' is not included in the joint group.");
      return false;
    }
    efforts_msg.commands.emplace_back();
    efforts_msg.commands.back().name = jnt_name;
    efforts_msg.commands.back().data = efforts(jnt_parser_.jointIndex(jnt_name));
  }

  return true;
}

bool EffortControllerNode::taskSpaceControl(
  const tobas_msgs::msg::JointStateArray& cur_js,
  const tobas_msgs::LinkStateArray& tar_ls,
  tobas_msgs::msg::JointCommandArray& efforts_msg)
{
  const auto active_jnt_names = findActiveJointNames(tree_, linkNames(tar_ls));
  if (!active_jnt_names) {
    TOBAS_ERROR("Failed to extract active joint names: ", active_jnt_names.error());
    return false;
  }

  // JointState -> JntArray
  if (const auto result = cur_js_conv_.convert(cur_js); !result) {
    TOBAS_ERROR("Failed to convert current JointState to Jntarray: ", result.error());
    return false;
  }

  // Update task-space target values.
  kdl::Frame T_Base_Parent;
  kdl::FrameMap tar_p;
  kdl::TwistMap tar_v;
  kdl::AccelMap a_ff;
  kdl::WrenchMap f_ext;
  for (const auto& ls : tar_ls.states) {
    const auto transform = tf_listener_->lookupTransform(tree_.getRootName(), tar_ls.header.frame_id);
    if (!transform) {
      TOBAS_ERROR(transform.error());
      continue;
    }

    // Convert values expressed in the parent frame to values expressed in the base link.
    kdl::transformMsgToKDL(transform->transform, T_Base_Parent);
    tar_p[ls.name] = T_Base_Parent * ls.frame;
    tar_v[ls.name] = T_Base_Parent.M * ls.twist;
    a_ff[ls.name] = T_Base_Parent.M * ls.accel;
    f_ext[ls.name] = T_Base_Parent.M * ls.wrench;
  }

  // Calculate joint torques with PID.
  const auto& cur_q = cur_js_conv_.getPosition();
  const auto& cur_qd = cur_js_conv_.getVelocity();
  const auto efforts = pid_ts_.cartToJnt(cur_q, cur_qd, tar_p, tar_v, a_ff, f_ext);
  if (!efforts) {
    TOBAS_ERROR("Failed to calculate target joint efforts: ", efforts.error());
    return false;
  }

  // Fill output message.
  for (const auto& jnt_name : active_jnt_names.value()) {
    if (!jnt_names_.contains(jnt_name)) {
      TOBAS_ERROR("The target joint '", jnt_name, "' is not included in the joint group.");
      return false;
    }
    efforts_msg.commands.emplace_back();
    efforts_msg.commands.back().name = jnt_name;
    efforts_msg.commands.back().data = efforts.value()(jnt_parser_.jointIndex(jnt_name));
  }

  return true;
}

void EffortControllerNode::jointStiffnessCb(const long& p)
{
  pid_js_.setStiffness(p);
}

void EffortControllerNode::jointDamping(const long& p)
{
  pid_js_.setDamping(p);
}

void EffortControllerNode::linearStiffnessCb(const long& p)
{
  pid_ts_.setLinearStiffness(p);
}

void EffortControllerNode::angularStiffnessCb(const long& p)
{
  pid_ts_.setAngularStiffness(p);
}

void EffortControllerNode::linearDampingCb(const long& p)
{
  pid_ts_.setLinearDamping(p);
}

void EffortControllerNode::angularDampingCb(const long& p)
{
  pid_ts_.setAngularDamping(p);
}

void EffortControllerNode::droneCb(const Drone::ConstSharedPtr& drone)
{
  drone_ = drone;

  home_js_.states.clear();

  // Get joint home positions.
  for (const auto& jnt_name : jnt_names_) {
    const auto joint_it = drone->joints.find(jnt_name);
    if (joint_it == drone->joints.end()) {
      TOBAS_WARN("The drone does not have joint '", jnt_name, "'.");
      continue;
    }
    const auto& joint = joint_it->second;
    if (joint.cmd_iface != JointCommandInterface::kEffort) {
      TOBAS_WARN("The command interface of joint '", jnt_name, "' is not effort.");
      continue;
    }
    home_js_.states.emplace_back();
    home_js_.states.back().name = jnt_name;
    home_js_.states.back().position = joint.home_pos;
  }

  // Set home positions as the initial target state.
  if (!home_js_.states.empty()) {
    tar_js_ = std::make_shared<tobas_msgs::msg::JointStateArray>(home_js_);
  }
}

void EffortControllerNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;

  jnt_parser_.updateInternalDataStructures();
  pid_js_.updateInternalDataStructures();
  pid_ts_.updateInternalDataStructures();
  cur_js_conv_.updateInternalDataStructures();
  tar_js_conv_.updateInternalDataStructures();
}

void EffortControllerNode::currentJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& cur_js)
{
  if (tree_.empty()) {
    return;
  }
  if (home_js_.states.empty()) {
    return;
  }
  if (!tar_js_ && !tar_ls_) {
    return;
  }

  // Create joint efforts command.
  auto efforts_msg = std::make_unique<tobas_msgs::msg::JointCommandArray>();
  efforts_msg->header.stamp = cur_js->header.stamp;

  // Joint space control or Task space control
  if (tar_js_) {
    if (!jointSpaceControl(*cur_js, *tar_js_, *efforts_msg)) {
      return;
    }
  }
  else if (tar_ls_) {
    if (!taskSpaceControl(*cur_js, *tar_ls_, *efforts_msg)) {
      return;
    }
  }
  else {
    TOBAS_ERROR("Both target joint state and target cartesian state are null.");
    return;
  }

  // Publish joint efforts command.
  efforts_pub_->publish(std::move(efforts_msg));
}

void EffortControllerNode::targetJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& tar_js)
{
  tar_js_ = tar_js;
  tar_ls_.reset();

  auto_reset_timer_->reset();
}

void EffortControllerNode::targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls)
{
  tar_ls_ = tar_ls;
  tar_js_.reset();

  auto_reset_timer_->reset();
}

void EffortControllerNode::autoResetTimerCb()
{
  tar_js_ = std::make_shared<tobas_msgs::msg::JointStateArray>(home_js_);
  tar_ls_.reset();

  TOBAS_WARN(
    "The target joint states are automatically reset because ",
    kAutoResetTimeThresh,
    " have elapsed since the last command.");

  auto_reset_timer_->cancel();
}
}  // namespace manipulation
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::manipulation::EffortControllerNode)
