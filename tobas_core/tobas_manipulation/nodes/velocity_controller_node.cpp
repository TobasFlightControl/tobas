#include <tobas_constants/ros_interface.hpp>
#include <tobas_kdl/tree_active_joints_extractor.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_kdl/tree_taskspace_vel_ctrl.hpp>
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
class VelocityControllerNode : public BaseNode
{
  using self = VelocityControllerNode;
  using super = BaseNode;

public:
  explicit VelocityControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Parameters
  std::unordered_set<std::string> jnt_names_;

  Drone::ConstSharedPtr drone_;
  kdl::Tree tree_;

  kdl::TreeJointParser jnt_parser_;
  kdl::TreeActiveJointsExtractor active_jnts_extractor_;
  kdl::TreeTaskSpaceVelCtrl vel_ctrl_;
  TreeJointStateConverter cur_js_conv_;
  TreeJointStateConverter tar_js_conv_;

  ros2::TransformListener::SharedPtr tf_listener_;
  double jnt_time_const_;
  tobas_msgs::msg::JointStateArray home_js_;

  tobas_msgs::msg::JointStateArray::ConstSharedPtr tar_js_;
  tobas_msgs::LinkStateArray::ConstSharedPtr tar_ls_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> velocities_pub_;

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
    tobas_msgs::msg::JointCommandArray& velocities_msg);
  bool taskSpaceControl(
    const tobas_msgs::msg::JointStateArray& cur_js,
    const tobas_msgs::LinkStateArray& tar_ls,
    tobas_msgs::msg::JointCommandArray& velocities_msg);

  bool jointTimeConstCb(const double& p);
  bool linearTimeConstCb(const double& p);
  bool angularTimeConstCb(const double& p);

  void droneCb(const Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void currentJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& cur_js);
  void targetJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& tar_js);
  void targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls);

  void autoResetTimerCb();
};

VelocityControllerNode::VelocityControllerNode(const rclcpp::NodeOptions& options)
  : super("jointvel_trajectory_controller", nodeOptions_DParam(options))
  , jnt_parser_(tree_)
  , active_jnts_extractor_(tree_)
  , vel_ctrl_(tree_)
  , cur_js_conv_(tree_)
  , tar_js_conv_(tree_)
{
  initialize_timer_ = createTimer(0s, &self::initialize, this);
}

void VelocityControllerNode::initialize()
{
  const auto jnt_names = getStringArrayParam("joint_names", {});
  if (jnt_names.empty()) {
    TOBAS_ERROR("Joint names are not specified.");
    return;
  }
  jnt_names_.insert(jnt_names.begin(), jnt_names.end());

  // shared_from_thisはコンストラクタでは呼べない
  tf_listener_ = std::make_shared<ros2::TransformListener>(shared_from_this());

  addDynamicDoubleParam("joint_time_constant", &self::jointTimeConstCb, this, 0.1, 3, 1, 10, " s");
  addDynamicDoubleParam("linear_time_constant", &self::linearTimeConstCb, this, 0.1, 5, 1, 10, " s");
  addDynamicDoubleParam("angular_time_constant", &self::angularTimeConstCb, this, 0.1, 5, 1, 10, " s");

  velocities_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(topic::kJointVelCmd);

  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(topic::kKdlTree, &self::treeCb, this, true, true);
  cur_js_sub_ = createSubscriber(topic::kJointStates, &self::currentJointStateCb, this);
  tar_js_sub_ = createSubscriber(topic::kVelCtrlJS, &self::targetJointStateCb, this);
  tar_ls_sub_ = createSubscriber(topic::kVelCtrlLS, &self::targetLinkStateCb, this);

  auto_reset_timer_ = createTimer(manipulation::kAutoResetTimeThresh, &self::autoResetTimerCb, this, false);

  initialize_timer_->cancel();
}

bool VelocityControllerNode::jointSpaceControl(
  const tobas_msgs::msg::JointStateArray& cur_js,
  const tobas_msgs::msg::JointStateArray& tar_js,
  tobas_msgs::msg::JointCommandArray& velocities_msg)
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

  // 目標関節速度を計算
  const auto& cur_q = cur_js_conv_.getPosition();
  const auto& tar_q = tar_js_conv_.getPosition();
  const auto gain = 1 / jnt_time_const_;
  const auto velocities = gain * (tar_q - cur_q);

  // TODO: 関節角制限を考慮し，制限に違反する速度を出さない

  // Fill output message
  for (const auto& tar_state : tar_js.states) {
    const auto& jnt_name = tar_state.name;
    if (!jnt_names_.contains(jnt_name)) {
      TOBAS_ERROR("The target joint \"", jnt_name, "\" is not included in the joint group.");
      return false;
    }
    velocities_msg.commands.emplace_back();
    velocities_msg.commands.back().name = jnt_name;
    velocities_msg.commands.back().data = velocities(jnt_parser_.jointIndex(jnt_name));
  }

  return true;
}

bool VelocityControllerNode::taskSpaceControl(
  const tobas_msgs::msg::JointStateArray& cur_js,
  const tobas_msgs::LinkStateArray& tar_ls,
  tobas_msgs::msg::JointCommandArray& velocities_msg)
{
  // JointState -> JntArray
  if (cur_js_conv_.convert(cur_js) < 0) {
    TOBAS_ERROR("Failed to convert current JointState to Jntarray: ", cur_js_conv_.errorMessage());
    return false;
  }

  kdl::Frame T_Base_Parent;
  kdl::FrameMap tar_p;
  for (const auto& ls : tar_ls.states) {
    if (!tf_listener_->lookupTransform(tree_.getRootName(), tar_ls.header.frame_id)) {
      TOBAS_ERROR(tf_listener_->getErrorMessage());
      continue;
    }

    kdl::transformMsgToKDL(tf_listener_->getTransform().transform, T_Base_Parent);
    tar_p[ls.name] = T_Base_Parent * ls.frame;  // Base -> Segment tip
  }

  // 目標関節速度を計算
  const auto& cur_q = cur_js_conv_.getPosition();
  if (vel_ctrl_.cartToJnt(cur_q, tar_p) < 0) {
    TOBAS_ERROR("Cartesian controller failed: ", vel_ctrl_.errorMessage());
    return false;
  }
  const auto& velocities = vel_ctrl_.getVelocities();

  // JntArray -> JointState
  active_jnts_extractor_.solve(manipulation::linkNames(tar_ls));
  const auto& active_jnt_names = active_jnts_extractor_.activeJointNames();

  // Fill output message
  for (const auto& jnt_name : active_jnt_names) {
    if (!jnt_names_.contains(jnt_name)) {
      TOBAS_ERROR("The target joint \"", jnt_name, "\" is not included in the joint group.");
      return false;
    }
    velocities_msg.commands.emplace_back();
    velocities_msg.commands.back().name = jnt_name;
    velocities_msg.commands.back().data = velocities(jnt_parser_.jointIndex(jnt_name));
  }

  return true;
}

bool VelocityControllerNode::jointTimeConstCb(const double& p)
{
  jnt_time_const_ = p;
  return true;
}

bool VelocityControllerNode::linearTimeConstCb(const double& p)
{
  if (!vel_ctrl_.setLinearTimeConst(p)) {
    TOBAS_ERROR("Failed to set linear tracking time constant.");
    return false;
  }

  return true;
}

bool VelocityControllerNode::angularTimeConstCb(const double& p)
{
  if (!vel_ctrl_.setAngularTimeConst(p)) {
    TOBAS_ERROR("Failed to set angular tracking time constant.");
    return false;
  }

  return true;
}

void VelocityControllerNode::droneCb(const Drone::ConstSharedPtr& drone)
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
    if (joint.cmd_iface != JointCommandInterface::kVelocity) {
      TOBAS_WARN("The command interface of joint \"", jnt_name, "\" is not velocity.");
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

void VelocityControllerNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
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
  if (!vel_ctrl_.updateInternalDataStructures()) {
    TOBAS_ERROR("Failed to update internal data structures of joint space velocity controller.");
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

void VelocityControllerNode::currentJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& cur_js)
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

  // Create joint velocities command
  auto velocities_msg = std::make_unique<tobas_msgs::msg::JointCommandArray>();
  velocities_msg->header.stamp = cur_js->header.stamp;

  // Joint space control or Task space control
  if (tar_js_) {
    if (!jointSpaceControl(*cur_js, *tar_js_, *velocities_msg)) {
      return;
    }
  }
  else if (tar_ls_) {
    if (!taskSpaceControl(*cur_js, *tar_ls_, *velocities_msg)) {
      return;
    }
  }
  else {
    TOBAS_ERROR("Both target joint state and target link state are null.");
    return;
  }

  // Publish joint velocities command
  velocities_pub_->publish(std::move(velocities_msg));
}

void VelocityControllerNode::targetJointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& tar_js)
{
  tar_js_ = tar_js;
  tar_ls_.reset();

  auto_reset_timer_->reset();
}

void VelocityControllerNode::targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls)
{
  tar_ls_ = tar_ls;
  tar_js_.reset();

  auto_reset_timer_->reset();
}

void VelocityControllerNode::autoResetTimerCb()
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

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::manipulation::VelocityControllerNode)
