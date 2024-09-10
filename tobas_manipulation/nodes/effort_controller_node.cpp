#include <sensor_msgs/msg/joint_state.hpp>

#include <tobas_std_tools/zip.hpp>
#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_kdl/treeactivejointsextractor.hpp>
#include <tobas_kdl/treejntspacepid.hpp>
#include <tobas_kdl/treetaskspacepid.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_ros2_tools/tf_listener.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs_adapter/LinkStateArray.hpp>
#include <tobas_kdl_msgs_adapter/Tree.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

#include "../include/tobas_manipulation/util.hpp"

using namespace std;

class EffortControllerNode : public tobas::BaseNode
{
  using self = EffortControllerNode;
  using super = tobas::BaseNode;

public:
  explicit EffortControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;
  kdl::Tree tree_;

  kdl::TreeJointStateConverter cur_js_conv_;
  kdl::TreeJointStateConverter tar_js_conv_;
  kdl::TreeActiveJointsExtractor active_jnts_extractor_;
  kdl::TreeJntSpacePID pid_js_;
  kdl::TreeTaskSpacePID pid_ts_;

  bool is_initialized_ = false;
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool is_commanded_ = false;
  ros2::TransformListener::SharedPtr tf_listener_;
  sensor_msgs::msg::JointState home_js_;
  rclcpp::Time t_last_cmd_;

  sensor_msgs::msg::JointState::ConstSharedPtr tar_js_;
  tobas_msgs::LinkStateArray::ConstSharedPtr tar_ls_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> efforts_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<sensor_msgs::msg::JointState> cur_js_sub_;
  ros2::SubscriberPtr<sensor_msgs::msg::JointState> tar_js_sub_;
  ros2::SubscriberPtr<tobas_msgs::LinkStateArray> tar_ls_sub_;

  void initialize();

  bool jointSpaceControl(
    const sensor_msgs::msg::JointState& cur_js,
    const sensor_msgs::msg::JointState& tar_js,
    tobas_msgs::msg::JointCommandArray& efforts_msg);
  bool taskSpaceControl(
    const sensor_msgs::msg::JointState& cur_js,
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
  void currentJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& cur_js);
  void targetJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& tar_js);
  void targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls);
};

EffortControllerNode::EffortControllerNode(const rclcpp::NodeOptions& options)
  : super("effort_controller", options),
    cur_js_conv_(tree_),
    tar_js_conv_(tree_),
    active_jnts_extractor_(tree_),
    pid_js_(tree_),
    pid_ts_(tree_)
{
  // Register dynamic parameters
  addDynamicDoubleParam("joint_stiffness", &self::jointStiffnessCb, this, 25., 0.1, 100.);
  addDynamicDoubleParam("joint_damping", &self::jointDamping, this, 10., 0.1, 20.);
  addDynamicDoubleParam("linear_stiffness", &self::linearStiffnessCb, this, 25., 0.1, 100.);
  addDynamicDoubleParam("angular_stiffness", &self::angularStiffnessCb, this, 25., 0.1, 100.);
  addDynamicDoubleParam("linear_damping", &self::linearDampingCb, this, 10., 0.1, 20.);
  addDynamicDoubleParam("angular_damping", &self::angularDampingCb, this, 10., 0.1, 20.);
  publishDynamicParameterDescriptions();

  // Register publishers
  efforts_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(tobas::kJointEffortsCmdTopic);

  // Register subscribers
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true);
  tree_sub_ = createSubscriber(tobas::kKDLTreeTopic, &self::treeCb, this, true);
  cur_js_sub_ = createSubscriber(tobas::kJointStatesTopic, &self::currentJointStateCb, this);
  tar_js_sub_ = createSubscriber(tobas::kEffCtrlJSTopic, &self::targetJointStateCb, this);
  tar_ls_sub_ = createSubscriber(tobas::kEffCtrlLSTopic, &self::targetLinkStateCb, this);
}

void EffortControllerNode::initialize()
{
  tf_listener_ = std::make_shared<ros2::TransformListener>(shared_from_this());

  cur_js_conv_.updateInternalDataStructures();
  tar_js_conv_.updateInternalDataStructures();
  active_jnts_extractor_.updateInternalDataStructures();
  pid_js_.updateInternalDataStructures();
  pid_ts_.updateInternalDataStructures();

  // 力指令タイプの関節のホームポジションを取得
  for (const auto& [jnt_name, jnt_cfg] : drone_.joints)
  {
    if (jnt_cfg.interface != tobas::joint_interface_t::EFFORT)
      continue;
    home_js_.name.push_back(jnt_name);
    home_js_.position.push_back(jnt_cfg.home_pos);
    home_js_.velocity.push_back(0.);
    home_js_.effort.push_back(0.);
  }

  // ホームポジションを初期目標状態に設定
  if (home_js_.name.size() > 0)
    tar_js_ = std::make_shared<sensor_msgs::msg::JointState>(home_js_);

  is_initialized_ = true;
}

bool EffortControllerNode::jointSpaceControl(
  const sensor_msgs::msg::JointState& cur_js,
  const sensor_msgs::msg::JointState& tar_js,
  tobas_msgs::msg::JointCommandArray& efforts_msg)
{
  // JointState -> JntArray
  if (cur_js_conv_.jointStateToJntArrayPosVel(cur_js) < 0)
  {
    TOBAS_ERROR("Failed to convert current JointState to Jntarray: ", cur_js_conv_.errorMessage());
    return false;
  }
  if (tar_js_conv_.jointStateToJntArrayPosVel(tar_js) < 0)
  {
    TOBAS_ERROR("Failed to convert target JointState to Jntarray: ", tar_js_conv_.errorMessage());
    return false;
  }

  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  const auto& cur_qd = cur_js_conv_.getVelocitiesKDL();
  const auto& tar_q = tar_js_conv_.getPositionsKDL();
  const auto& tar_qd = tar_js_conv_.getVelocitiesKDL();

  // PIDで関節トルクを計算
  if (pid_js_.CartToJnt(cur_q, cur_qd, tar_q, tar_qd) < 0)
  {
    TOBAS_ERROR("Joint space PID failed: ", pid_js_.errorMessage());
    return false;
  }
  const auto efforts = tar_js_conv_.getEffortsKDL() + pid_js_.getEfforts();  // FF + FB

  // JntArray -> JointState
  if (tar_js_conv_.jntArrayToJointStateEff(efforts, tar_js.name) < 0)
  {
    TOBAS_ERROR("Failed to convert Jntarray to JointState: ", tar_js_conv_.errorMessage());
    return false;
  }

  // Fill output message
  for (const auto& [name, eff] : tobas_std::zip(tar_js_conv_.getNamesMsg(), tar_js_conv_.getEffortsMsg()))
  {
    efforts_msg.commands.emplace_back();
    efforts_msg.commands.back().name = name;
    efforts_msg.commands.back().data = eff;
  }

  return true;
}

bool EffortControllerNode::taskSpaceControl(
  const sensor_msgs::msg::JointState& cur_js,
  const tobas_msgs::LinkStateArray& tar_ls,
  tobas_msgs::msg::JointCommandArray& efforts_msg)
{
  // JointState -> JntArray
  if (cur_js_conv_.jointStateToJntArrayPosVel(cur_js) < 0)
  {
    TOBAS_ERROR("Failed to convert current JointState to Jntarray: ", cur_js_conv_.errorMessage());
    return false;
  }

  // デカルト座標系の目標値を更新
  kdl::Frame T_Base_Parent;
  kdl::FrameMap tar_p;
  kdl::TwistMap tar_v;
  kdl::AccelMap a_ff;
  kdl::WrenchMap f_ext;
  for (const auto& ls : tar_ls.states)
  {
    if (!tf_listener_->lookupTransform(tree_.getRootName(), tar_ls.header.frame_id))
    {
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
  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  const auto& cur_qd = cur_js_conv_.getVelocitiesKDL();
  if (pid_ts_.CartToJnt(cur_q, cur_qd, tar_p, tar_v, a_ff, f_ext) < 0)
  {
    TOBAS_ERROR("Cartesian PID failed: ", pid_ts_.errorMessage());
    return false;
  }
  const auto& efforts = pid_ts_.getEfforts();

  // JntArray -> JointState
  active_jnts_extractor_.solve(manipulation::linkNames(tar_ls));
  const auto& active_joints = active_jnts_extractor_.activeJointNames();
  if (tar_js_conv_.jntArrayToJointStateEff(efforts, active_joints) < 0)
  {
    TOBAS_ERROR("Failed to convert Jntarray to JointState: ", tar_js_conv_.errorMessage());
    return false;
  }

  // Fill output message
  for (const auto& [name, eff] : tobas_std::zip(tar_js_conv_.getNamesMsg(), tar_js_conv_.getEffortsMsg()))
  {
    efforts_msg.commands.emplace_back();
    efforts_msg.commands.back().name = name;
    efforts_msg.commands.back().data = eff;
  }

  return true;
}

bool EffortControllerNode::jointStiffnessCb(const double& p)
{
  if (!pid_js_.setStiffness(p))
  {
    TOBAS_ERROR("Failed to set joint stiffness.");
    return false;
  }

  return true;
}

bool EffortControllerNode::jointDamping(const double& p)
{
  if (!pid_js_.setDamping(p))
  {
    TOBAS_ERROR("Failed to set joint damping.");
    return false;
  }

  return true;
}

bool EffortControllerNode::linearStiffnessCb(const double& p)
{
  if (!pid_ts_.setLinearStiffness(p))
  {
    TOBAS_ERROR("Failed to set linear stiffness.");
    return false;
  }

  return true;
}

bool EffortControllerNode::angularStiffnessCb(const double& p)
{
  if (!pid_ts_.setAngularStiffness(p))
  {
    TOBAS_ERROR("Failed to set angular stiffness.");
    return false;
  }

  return true;
}

bool EffortControllerNode::linearDampingCb(const double& p)
{
  if (!pid_ts_.setLinearDamping(p))
  {
    TOBAS_ERROR("Failed to set linear damping.");
    return false;
  }

  return true;
}

bool EffortControllerNode::angularDampingCb(const double& p)
{
  if (!pid_ts_.setAngularDamping(p))
  {
    TOBAS_ERROR("Failed to set angular damping.");
    return false;
  }

  return true;
}

void EffortControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = *drone;
  drone_received_ = true;

  if (tree_received_)
    initialize();
}

void EffortControllerNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;
  tree_received_ = true;

  if (drone_received_)
    initialize();
}

void EffortControllerNode::currentJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& cur_js)
{
  if (!is_initialized_)
    return;

  if (tar_js_ == nullptr && tar_ls_ == nullptr)
    return;

  const auto time_after_last_cmd = (get_clock()->now() - t_last_cmd_).seconds();
  if (is_commanded_ && time_after_last_cmd > tobas::kAutoResetTimeThreshold)
  {
    tar_js_ = std::make_shared<sensor_msgs::msg::JointState>(home_js_);
    tar_ls_ = nullptr;
    is_commanded_ = false;
    TOBAS_WARN(
      "The target joint states are automatically reset because ", tobas::kAutoResetTimeThreshold,
      " seconds have elapsed since the last command.");
  }

  // Create joint efforts command
  auto efforts_msg = std::make_unique<tobas_msgs::msg::JointCommandArray>();

  // Joint space control or Task space control
  if (tar_js_ != nullptr)
  {
    if (!jointSpaceControl(*cur_js, *tar_js_, *efforts_msg))
      return;
  }
  else if (tar_ls_ != nullptr)
  {
    if (!taskSpaceControl(*cur_js, *tar_ls_, *efforts_msg))
      return;
  }
  else
  {
    TOBAS_ERROR("Both target joint state and target cartesian state are NULL.");
    return;
  }

  // Publish joint efforts command
  efforts_pub_->publish(move(efforts_msg));
}

void EffortControllerNode::targetJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& tar_js)
{
  tar_js_ = tar_js;
  tar_ls_ = nullptr;

  t_last_cmd_ = get_clock()->now();
  is_commanded_ = true;
}

void EffortControllerNode::targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls)
{
  tar_ls_ = tar_ls;
  tar_js_ = nullptr;

  t_last_cmd_ = get_clock()->now();
  is_commanded_ = true;
}

RCLCPP_COMPONENTS_REGISTER_NODE(EffortControllerNode)
