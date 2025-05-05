#include <tobas_constants/constants.hpp>
#include <tobas_kdl/tree_active_joints_extractor.hpp>
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

#include "../include/tobas_manipulation/constants.hpp"
#include "../include/tobas_manipulation/util.hpp"

using namespace std;

class EffortControllerNode : public tobas::BaseNode
{
  using self = EffortControllerNode;
  using super = tobas::BaseNode;

public:
  explicit EffortControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone::ConstSharedPtr drone_;
  kdl::Tree tree_;

  kdl::TreeJointParser jnt_parser_;
  kdl::TreeActiveJointsExtractor active_jnts_extractor_;
  kdl::TreeJntSpacePID pid_js_;
  kdl::TreeTaskSpacePID pid_ts_;
  tobas::TreeJointStateConverter cur_js_conv_;
  tobas::TreeJointStateConverter tar_js_conv_;

  ros2::TransformListener::SharedPtr tf_listener_;
  tobas_msgs::msg::JointStateArray home_js_;

  tobas_msgs::msg::JointStateArray::ConstSharedPtr tar_js_;
  tobas_msgs::LinkStateArray::ConstSharedPtr tar_ls_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> efforts_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
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

  bool jointStiffnessCb(const double& p);
  bool jointDamping(const double& p);
  bool linearStiffnessCb(const double& p);
  bool angularStiffnessCb(const double& p);
  bool linearDampingCb(const double& p);
  bool angularDampingCb(const double& p);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void currentJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& cur_js);
  void targetJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& tar_js);
  void targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls);

  void autoResetTimerCb();
};

EffortControllerNode::EffortControllerNode(const rclcpp::NodeOptions& options)
  : super("effort_controller", options)
  , jnt_parser_(tree_)
  , active_jnts_extractor_(tree_)
  , pid_js_(tree_)
  , pid_ts_(tree_)
  , cur_js_conv_(tree_)
  , tar_js_conv_(tree_)
{
  initialize_timer_ = createTimer(0s, &self::initialize, this);
}

void EffortControllerNode::initialize()
{
  // shared_from_thisはコンストラクタでは呼べない
  tf_listener_ = std::make_shared<ros2::TransformListener>(shared_from_this());

  addDynamicDoubleParam("joint_stiffness", &self::jointStiffnessCb, this, 25., 0.1, 100.);
  addDynamicDoubleParam("joint_damping", &self::jointDamping, this, 10., 0.1, 20.);
  addDynamicDoubleParam("linear_stiffness", &self::linearStiffnessCb, this, 25., 0.1, 100.);
  addDynamicDoubleParam("angular_stiffness", &self::angularStiffnessCb, this, 25., 0.1, 100.);
  addDynamicDoubleParam("linear_damping", &self::linearDampingCb, this, 10., 0.1, 20.);
  addDynamicDoubleParam("angular_damping", &self::angularDampingCb, this, 10., 0.1, 20.);

  efforts_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(tobas::kJointEffCmdTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(tobas::kKdlTreeTopic, &self::treeCb, this, true, true);
  cur_js_sub_ = createSubscriber(tobas::kJointStatesTopic, &self::currentJointStateCb, this);
  tar_js_sub_ = createSubscriber(tobas::kEffCtrlJSTopic, &self::targetJointStateCb, this);
  tar_ls_sub_ = createSubscriber(tobas::kEffCtrlLSTopic, &self::targetLinkStateCb, this);

  auto_reset_timer_ = createTimer(manipulation::kAutoResetTimeThresh, &self::autoResetTimerCb, this, false);

  initialize_timer_->cancel();
}

bool EffortControllerNode::jointSpaceControl(
  const tobas_msgs::msg::JointStateArray& cur_js,
  const tobas_msgs::msg::JointStateArray& tar_js,
  tobas_msgs::msg::JointCommandArray& efforts_msg)
{
  // JointState -> JntArray
  if (cur_js_conv_.convert(cur_js) < 0) {
    TOBAS_ERROR("Failed to convert current JointState to Jntarray: ", cur_js_conv_.errorMessage());
    return false;
  }
  if (tar_js_conv_.convert(tar_js) < 0) {
    TOBAS_ERROR("Failed to convert target JointState to Jntarray: ", tar_js_conv_.errorMessage());
    return false;
  }

  const auto& cur_q = cur_js_conv_.getPosition();
  const auto& cur_qd = cur_js_conv_.getVelocity();
  const auto& tar_q = tar_js_conv_.getPosition();
  const auto& tar_qd = tar_js_conv_.getVelocity();

  // PIDで関節トルクを計算
  if (pid_js_.CartToJnt(cur_q, cur_qd, tar_q, tar_qd) < 0) {
    TOBAS_ERROR("Joint space PID failed: ", pid_js_.errorMessage());
    return false;
  }
  const auto efforts = tar_js_conv_.getEffort() + pid_js_.getEfforts();  // FF + FB

  // Fill output message
  for (const auto& tar_state : tar_js.states) {
    const auto& joint = drone_->joints.at(tar_state.name);
    if (joint.role != tobas::jnt_role_t::MANIPULATION) {
      TOBAS_WARN("The role of joint \"", tar_state.name, "\" must be \"MANIPULATION\".");
      continue;
    }
    if (joint.cmd_iface != tobas::jnt_cmd_iface_t::EFFORT) {
      TOBAS_WARN("The command interface of joint \"", tar_state.name, "\" must be \"EFFORT\".");
      continue;
    }

    efforts_msg.commands.emplace_back();
    efforts_msg.commands.back().name = tar_state.name;
    efforts_msg.commands.back().data = efforts(jnt_parser_.jointIndex(tar_state.name));
  }

  return true;
}

bool EffortControllerNode::taskSpaceControl(
  const tobas_msgs::msg::JointStateArray& cur_js,
  const tobas_msgs::LinkStateArray& tar_ls,
  tobas_msgs::msg::JointCommandArray& efforts_msg)
{
  // JointState -> JntArray
  if (cur_js_conv_.convert(cur_js) < 0) {
    TOBAS_ERROR("Failed to convert current JointState to Jntarray: ", cur_js_conv_.errorMessage());
    return false;
  }

  // デカルト座標系の目標値を更新
  kdl::Frame T_Base_Parent;
  kdl::FrameMap tar_p;
  kdl::TwistMap tar_v;
  kdl::AccelMap a_ff;
  kdl::WrenchMap f_ext;
  for (const auto& ls : tar_ls.states) {
    if (!tf_listener_->lookupTransform(tree_.getRootName(), tar_ls.header.frame_id)) {
      TOBAS_ERROR(tf_listener_->getErrorMessage());
      continue;
    }

    // 親フレームで表現された値をベースリンクで表現された値に変換
    kdl::transformMsgToKDL(tf_listener_->getTransform().transform, T_Base_Parent);
    tar_p[ls.name] = T_Base_Parent * ls.frame;
    tar_v[ls.name] = T_Base_Parent.M * ls.twist;
    a_ff[ls.name] = T_Base_Parent.M * ls.accel;
    f_ext[ls.name] = T_Base_Parent.M * ls.wrench;
  }

  // PIDで関節トルクを計算
  const auto& cur_q = cur_js_conv_.getPosition();
  const auto& cur_qd = cur_js_conv_.getVelocity();
  if (pid_ts_.CartToJnt(cur_q, cur_qd, tar_p, tar_v, a_ff, f_ext) < 0) {
    TOBAS_ERROR("Cartesian PID failed: ", pid_ts_.errorMessage());
    return false;
  }
  const auto& efforts = pid_ts_.getEfforts();

  // JntArray -> JointState
  active_jnts_extractor_.solve(manipulation::linkNames(tar_ls));
  const auto& active_jnt_names = active_jnts_extractor_.activeJointNames();

  // Fill output message
  for (const auto& jnt_name : active_jnt_names) {
    const auto& joint = drone_->joints.at(jnt_name);
    if (joint.role != tobas::jnt_role_t::MANIPULATION) {
      TOBAS_WARN("The role of joint \"", jnt_name, "\" must be \"MANIPULATION\".");
      continue;
    }
    if (joint.cmd_iface != tobas::jnt_cmd_iface_t::EFFORT) {
      TOBAS_WARN("The command interface of joint \"", jnt_name, "\" must be \"EFFORT\".");
      continue;
    }

    efforts_msg.commands.emplace_back();
    efforts_msg.commands.back().name = jnt_name;
    efforts_msg.commands.back().data = efforts((jnt_parser_.jointIndex(jnt_name)));
  }

  return true;
}

bool EffortControllerNode::jointStiffnessCb(const double& p)
{
  if (!pid_js_.setStiffness(p)) {
    TOBAS_ERROR("Failed to set joint stiffness.");
    return false;
  }

  return true;
}

bool EffortControllerNode::jointDamping(const double& p)
{
  if (!pid_js_.setDamping(p)) {
    TOBAS_ERROR("Failed to set joint damping.");
    return false;
  }

  return true;
}

bool EffortControllerNode::linearStiffnessCb(const double& p)
{
  if (!pid_ts_.setLinearStiffness(p)) {
    TOBAS_ERROR("Failed to set linear stiffness.");
    return false;
  }

  return true;
}

bool EffortControllerNode::angularStiffnessCb(const double& p)
{
  if (!pid_ts_.setAngularStiffness(p)) {
    TOBAS_ERROR("Failed to set angular stiffness.");
    return false;
  }

  return true;
}

bool EffortControllerNode::linearDampingCb(const double& p)
{
  if (!pid_ts_.setLinearDamping(p)) {
    TOBAS_ERROR("Failed to set linear damping.");
    return false;
  }

  return true;
}

bool EffortControllerNode::angularDampingCb(const double& p)
{
  if (!pid_ts_.setAngularDamping(p)) {
    TOBAS_ERROR("Failed to set angular damping.");
    return false;
  }

  return true;
}

void EffortControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;

  home_js_.states.clear();

  // 力指令タイプの関節のホームポジションを取得
  for (const auto& [jnt_name, jnt_cfg] : drone->joints) {
    if (jnt_cfg.role != tobas::jnt_role_t::MANIPULATION) {
      continue;
    }
    if (jnt_cfg.cmd_iface != tobas::jnt_cmd_iface_t::EFFORT) {
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

void EffortControllerNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;

  if (!jnt_parser_.updateInternalDataStructures()) {
    TOBAS_ERROR("Failed to update internal data structures of joint parser.");
    tree_.clear();
    return;
  }
  if (!active_jnts_extractor_.updateInternalDataStructures()) {
    TOBAS_ERROR("Failed to update internal data structures of active joints extractor.");
    tree_.clear();
    return;
  }
  if (!pid_js_.updateInternalDataStructures()) {
    TOBAS_ERROR("Failed to update internal data structures of joint space PID.");
    tree_.clear();
    return;
  }
  if (!pid_ts_.updateInternalDataStructures()) {
    TOBAS_ERROR("Failed to update internal data structures of task space PID.");
    tree_.clear();
    return;
  }
  if (!cur_js_conv_.updateInternalDataStructures()) {
    TOBAS_ERROR("Failed to update internal data structures of the joint state converter for current joints.");
    tree_.clear();
    return;
  }
  if (!tar_js_conv_.updateInternalDataStructures()) {
    TOBAS_ERROR("Failed to update internal data structures of the joint state converter for target joints.");
    tree_.clear();
    return;
  }
}

void EffortControllerNode::currentJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& cur_js)
{
  if (tree_.getNrOfJoints() == 0) {
    return;
  }
  if (home_js_.states.size() == 0) {
    return;
  }
  if (!tar_js_ && !tar_ls_) {
    return;
  }

  // Create joint efforts command
  auto efforts_msg = std::make_unique<tobas_msgs::msg::JointCommandArray>();

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
    TOBAS_ERROR("Both target joint state and target cartesian state are NULL.");
    return;
  }

  // Publish joint efforts command
  efforts_pub_->publish(move(efforts_msg));
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
    manipulation::kAutoResetTimeThresh,
    " have elapsed since the last command.");

  auto_reset_timer_->cancel();
}

RCLCPP_COMPONENTS_REGISTER_NODE(EffortControllerNode)
