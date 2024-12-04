#include <ranges>

#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_kdl/treeactivejointsextractor.hpp>
#include <tobas_kdl/treetaskspacevelctrl.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_ros2_tools/tf_listener.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs_adapter/link_state_array.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

#include "../include/tobas_manipulation/constants.hpp"
#include "../include/tobas_manipulation/util.hpp"

using namespace std;

class VelocityControllerNode : public tobas::BaseNode
{
  using self = VelocityControllerNode;
  using super = tobas::BaseNode;

public:
  explicit VelocityControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone::ConstSharedPtr drone_;
  kdl::Tree tree_;

  kdl::TreeJointStateConverter cur_js_conv_;
  kdl::TreeJointStateConverter tar_js_conv_;
  kdl::TreeActiveJointsExtractor active_jnts_extractor_;
  kdl::TreeTaskSpaceVelCtrl vel_ctrl_;

  ros2::TransformListener::SharedPtr tf_listener_;
  double jnt_time_const_;
  sensor_msgs::msg::JointState home_js_;

  sensor_msgs::msg::JointState::ConstSharedPtr tar_js_;
  tobas_msgs::LinkStateArray::ConstSharedPtr tar_ls_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> velocities_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<sensor_msgs::msg::JointState> cur_js_sub_;
  ros2::SubscriberPtr<sensor_msgs::msg::JointState> tar_js_sub_;
  ros2::SubscriberPtr<tobas_msgs::LinkStateArray> tar_ls_sub_;

  // Timer
  ros2::TimerPtr initialize_timer_;
  ros2::TimerPtr auto_reset_timer_;

  void initialize();

  bool jointSpaceControl(
    const sensor_msgs::msg::JointState& cur_js,
    const sensor_msgs::msg::JointState& tar_js,
    tobas_msgs::msg::JointCommandArray& velocities_msg);
  bool taskSpaceControl(
    const sensor_msgs::msg::JointState& cur_js,
    const tobas_msgs::LinkStateArray& tar_ls,
    tobas_msgs::msg::JointCommandArray& velocities_msg);

  bool jointTimeConstCb(const double& p);
  bool linearTimeConstCb(const double& p);
  bool angularTimeConstCb(const double& p);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void currentJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& cur_js);
  void targetJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& tar_js);
  void targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls);

  void autoResetTimerCb();
};

VelocityControllerNode::VelocityControllerNode(const rclcpp::NodeOptions& options)
  : super("velocity_controller", options),
    cur_js_conv_(tree_),
    tar_js_conv_(tree_),
    active_jnts_extractor_(tree_),
    vel_ctrl_(tree_)
{
  initialize_timer_ = createTimer(0s, &self::initialize, this);
}

void VelocityControllerNode::initialize()
{
  // shared_from_thisはコンストラクタでは呼べない
  tf_listener_ = std::make_shared<ros2::TransformListener>(shared_from_this());

  addDynamicDoubleParam("joint_time_constant", &self::jointTimeConstCb, this, 0.3, 0.01, 1.);
  addDynamicDoubleParam("linear_time_constant", &self::linearTimeConstCb, this, 0.5, 0.01, 1.);
  addDynamicDoubleParam("angular_time_constant", &self::angularTimeConstCb, this, 0.5, 0.01, 1.);

  velocities_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(tobas::kJointVelocitiesCmdTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(tobas::kKDLTreeTopic, &self::treeCb, this, true, true);
  cur_js_sub_ = createSubscriber(tobas::kJointStatesTopic, &self::currentJointStateCb, this);
  tar_js_sub_ = createSubscriber(tobas::kVelCtrlJSTopic, &self::targetJointStateCb, this);
  tar_ls_sub_ = createSubscriber(tobas::kVelCtrlLSTopic, &self::targetLinkStateCb, this);

  auto_reset_timer_ = createTimer(manipulation::kAutoResetTimeThresh, &self::autoResetTimerCb, this, false);

  initialize_timer_->cancel();
}

bool VelocityControllerNode::jointSpaceControl(
  const sensor_msgs::msg::JointState& cur_js,
  const sensor_msgs::msg::JointState& tar_js,
  tobas_msgs::msg::JointCommandArray& velocities_msg)
{
  // JointState -> JntArray
  if (cur_js_conv_.jointStateToJntArrayPos(cur_js) < 0)
  {
    TOBAS_ERROR("Failed to convert current JointState to Jntarray: ", cur_js_conv_.errorMessage());
    return false;
  }
  if (tar_js_conv_.jointStateToJntArrayPos(tar_js) < 0)
  {
    TOBAS_ERROR("Failed to convert target JointState to Jntarray: ", tar_js_conv_.errorMessage());
    return false;
  }

  // 目標関節速度を計算
  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  const auto& tar_q = tar_js_conv_.getPositionsKDL();
  const auto gain = 1 / jnt_time_const_;
  const auto velocities = gain * (tar_q - cur_q);

  // TODO: 関節角制限を考慮し，制限に違反する速度を出さない

  // JntArray -> JointState
  if (tar_js_conv_.jntArrayToJointStateVel(velocities, tar_js.name) < 0)
  {
    TOBAS_ERROR("Failed to convert Jntarray to JointState: ", tar_js_conv_.errorMessage());
    return false;
  }

  // Fill output message
  for (const auto& [name, vel] : views::zip(tar_js_conv_.getNamesMsg(), tar_js_conv_.getVelocitiesMsg()))
  {
    const auto& joint = drone_->joints.at(name);
    if (joint.interface != tobas::joint_interface_t::VELOCITY)
    {
      TOBAS_WARN("The command interface of joint \"", name, "\" must be \"VELOCITY\".");
      continue;
    }
    if (joint.role != tobas::joint_role_t::MANIPULATION)
    {
      TOBAS_WARN("The role of joint \"", name, "\" must be \"MANIPULATION\".");
      continue;
    }

    velocities_msg.commands.emplace_back();
    velocities_msg.commands.back().name = name;
    velocities_msg.commands.back().data = vel;
  }

  return true;
}

bool VelocityControllerNode::taskSpaceControl(
  const sensor_msgs::msg::JointState& cur_js,
  const tobas_msgs::LinkStateArray& tar_ls,
  tobas_msgs::msg::JointCommandArray& velocities_msg)
{
  // JointState -> JntArray
  if (cur_js_conv_.jointStateToJntArrayPos(cur_js) < 0)
  {
    TOBAS_ERROR("Failed to convert current JointState to Jntarray: ", cur_js_conv_.errorMessage());
    return false;
  }

  kdl::Frame T_Base_Parent;
  kdl::FrameMap tar_p;
  for (const auto& ls : tar_ls.states)
  {
    if (!tf_listener_->lookupTransform(tree_.getRootName(), tar_ls.header.frame_id))
    {
      TOBAS_ERROR(tf_listener_->getErrorMessage());
      continue;
    }

    kdl::transformMsgToKDL(tf_listener_->getTransform().transform, T_Base_Parent);
    tar_p[ls.name] = T_Base_Parent * ls.frame;  // Base -> Segment tip
  }

  // 目標関節速度を計算
  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  if (vel_ctrl_.CartToJnt(cur_q, tar_p) < 0)
  {
    TOBAS_ERROR("Cartesian controller failed: ", vel_ctrl_.errorMessage());
    return false;
  }
  const auto& velocities = vel_ctrl_.getVelocities();

  // JntArray -> JointState
  active_jnts_extractor_.solve(manipulation::linkNames(tar_ls));
  const auto& active_joints = active_jnts_extractor_.activeJointNames();
  if (tar_js_conv_.jntArrayToJointStateVel(velocities, active_joints) < 0)
  {
    TOBAS_ERROR("Failed to convert Jntarray to JointState: ", tar_js_conv_.errorMessage());
    return false;
  }

  // Fill output message
  for (const auto& [name, vel] : views::zip(tar_js_conv_.getNamesMsg(), tar_js_conv_.getVelocitiesMsg()))
  {
    const auto& joint = drone_->joints.at(name);
    if (joint.interface != tobas::joint_interface_t::VELOCITY)
    {
      TOBAS_WARN("The command interface of joint \"", name, "\" must be \"VELOCITY\".");
      continue;
    }
    if (joint.role != tobas::joint_role_t::MANIPULATION)
    {
      TOBAS_WARN("The role of joint \"", name, "\" must be \"MANIPULATION\".");
      continue;
    }

    velocities_msg.commands.emplace_back();
    velocities_msg.commands.back().name = name;
    velocities_msg.commands.back().data = vel;
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
  if (!vel_ctrl_.setLinearTimeConst(p))
  {
    TOBAS_ERROR("Failed to set linear tracking time constant.");
    return false;
  }

  return true;
}

bool VelocityControllerNode::angularTimeConstCb(const double& p)
{
  if (!vel_ctrl_.setAngularTimeConst(p))
  {
    TOBAS_ERROR("Failed to set angular tracking time constant.");
    return false;
  }

  return true;
}

void VelocityControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;

  home_js_.name.clear();
  home_js_.position.clear();
  home_js_.velocity.clear();
  home_js_.effort.clear();

  // 速度指令タイプの関節のホームポジションを取得
  for (const auto& [jnt_name, jnt_cfg] : drone->joints)
  {
    if (jnt_cfg.interface != tobas::joint_interface_t::VELOCITY)
      continue;
    if (jnt_cfg.role != tobas::joint_role_t::MANIPULATION)
      continue;
    home_js_.name.push_back(jnt_name);
    home_js_.position.push_back(jnt_cfg.home_pos);
    home_js_.velocity.push_back(0.);
    home_js_.effort.push_back(0.);
  }

  // ホームポジションを初期目標状態に設定
  if (home_js_.name.size() > 0)
    tar_js_ = std::make_shared<sensor_msgs::msg::JointState>(home_js_);
}

void VelocityControllerNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;

  cur_js_conv_.updateInternalDataStructures();
  tar_js_conv_.updateInternalDataStructures();
  active_jnts_extractor_.updateInternalDataStructures();
  vel_ctrl_.updateInternalDataStructures();
}

void VelocityControllerNode::currentJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& cur_js)
{
  if (tree_.getNrOfJoints() == 0)
    return;
  if (home_js_.name.size() == 0)
    return;
  if (tar_js_ == nullptr && tar_ls_ == nullptr)
    return;

  // Create joint velocities command
  auto velocities_msg = std::make_unique<tobas_msgs::msg::JointCommandArray>();

  // Joint space control or Task space control
  if (tar_js_ != nullptr)
  {
    if (!jointSpaceControl(*cur_js, *tar_js_, *velocities_msg))
      return;
  }
  else if (tar_ls_ != nullptr)
  {
    if (!taskSpaceControl(*cur_js, *tar_ls_, *velocities_msg))
      return;
  }
  else
  {
    TOBAS_ERROR("Both target joint state and target link state are NULL.");
    return;
  }

  // Publish joint velocities command
  velocities_pub_->publish(move(velocities_msg));
}

void VelocityControllerNode::targetJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& tar_js)
{
  tar_js_ = tar_js;
  tar_ls_ = nullptr;

  auto_reset_timer_->reset();
}

void VelocityControllerNode::targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls)
{
  tar_ls_ = tar_ls;
  tar_js_ = nullptr;

  auto_reset_timer_->reset();
}

void VelocityControllerNode::autoResetTimerCb()
{
  tar_js_ = std::make_shared<sensor_msgs::msg::JointState>(home_js_);
  tar_ls_ = nullptr;

  TOBAS_WARN(
    "The target joint states are automatically reset because ", manipulation::kAutoResetTimeThresh,
    " seconds have elapsed since the last command.");

  auto_reset_timer_->cancel();
}

RCLCPP_COMPONENTS_REGISTER_NODE(VelocityControllerNode)
