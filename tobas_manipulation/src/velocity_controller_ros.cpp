#include <tobas_std_tools/zip.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_manipulation/velocity_controller_ros.hpp"
#include "../include/tobas_manipulation/util.hpp"

using namespace std;

namespace tobas_manipulation
{
VelocityControllerRos::VelocityControllerRos(const rclcpp::NodeOptions& options)
  : super(node, pnh, name),
    cur_js_conv_(tree_),
    tar_js_conv_(tree_),
    active_jnts_extractor_(tree_),
    vel_ctrl_(tree_),

{


  cur_js_conv_.updateInternalDataStructures();
  tar_js_conv_.updateInternalDataStructures();
  active_jnts_extractor_.updateInternalDataStructures();
  vel_ctrl_.updateInternalDataStructures();

  // 速度指令タイプの関節のホームポジションを取得
  for (const auto& [jnt_name, jnt_cfg] : drone_.joints)
  {
    if (jnt_cfg.control_type != tobas::JointConfig::VELOCITY)
      continue;
    home_js_.name.push_back(jnt_name);
    home_js_.position.push_back(jnt_cfg.home_pos);
    home_js_.velocity.push_back(0.);
    home_js_.effort.push_back(0.);
  }

  // ホームポジションを初期目標状態に設定
  if (home_js_.name.size() > 0)
    tar_js_ =std::make_unique<sensor_msgs::msg::JointState>(home_js_);

  velocities_pub_ = createPublisher<tobas_msgs::JointCommandArray>(tobas::kJointVelocitiesCmdTopic);

  cur_js_sub_ = createSubscriber(tobas::kJointStatesTopic, &self::currentJointStateCb, this);
  tar_js_sub_ = createSubscriber(tobas::kVelCtrlJSTopic, &self::targetJointStateCb, this);
  tar_ls_sub_ = createSubscriber(tobas::kVelCtrlLSTopic, &self::targetLinkStateCb, this);


}

int VelocityControllerRos::jointSpaceControl(tobas_msgs::JointCommandArray& velocities_msg)
{
  // JointState -> JntArray
  if (cur_js_conv_.jointStateToJntArrayPos(*cur_js_) < 0)
  {
    TOBAS_ERROR("Failed to convert current JointState to Jntarray: ", cur_js_conv_.errorMessage());
    return -1;
  }
  if (tar_js_conv_.jointStateToJntArrayPos(*tar_js_) < 0)
  {
    TOBAS_ERROR("Failed to convert target JointState to Jntarray: ", tar_js_conv_.errorMessage());
    return -1;
  }

  // 目標関節速度を計算
  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  const auto& tar_q = tar_js_conv_.getPositionsKDL();
  const auto gain = 1 / jnt_time_const_;
  const auto velocities = gain * (tar_q - cur_q);

  // TODO: 関節角制限を考慮し，制限に違反する速度を出さない

  // JntArray -> JointState
  if (tar_js_conv_.jntArrayToJointStateVel(velocities, tar_js_->name) < 0)
  {
    TOBAS_ERROR("Failed to convert Jntarray to JointState: ", tar_js_conv_.errorMessage());
    return -1;
  }

  // Fill output message
  for (const auto& [name, vel] : tobas_std::zip(tar_js_conv_.getNamesMsg(), tar_js_conv_.getVelocitiesMsg()))
    velocities_msg.commands.emplace_back(name, vel);

  return 0;
}

int VelocityControllerRos::taskSpaceControl(tobas_msgs::JointCommandArray& velocities_msg)
{
  // JointState -> JntArray
  if (cur_js_conv_.jointStateToJntArrayPos(*cur_js_) < 0)
  {
    TOBAS_ERROR("Failed to convert current JointState to Jntarray: ", cur_js_conv_.errorMessage());
    return -1;
  }

  kdl::Frame T_Base_Parent;
  kdl::FrameMap tar_p;
  for (const auto& ls : tar_ls_->states)
  {
    if (!tf_listener_.lookupTransform(tree_.getRootName(), tar_ls_->header.frame_id))
    {
      TOBAS_ERROR(tf_listener_.getErrorMessage());
      continue;
    }

    transformMsgToKDL(tf_listener_.getTransform().transform, T_Base_Parent);
    tar_p[ls.name] = T_Base_Parent * ls.frame;  // Base -> Segment tip
  }

  // 目標関節速度を計算
  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  if (vel_ctrl_.CartToJnt(cur_q, tar_p) < 0)
  {
    TOBAS_ERROR("Cartesian controller failed: ", vel_ctrl_.errorMessage());
    return -1;
  }
  const auto& velocities = vel_ctrl_.getVelocities();

  // JntArray -> JointState
  active_jnts_extractor_.solve(linkNames(*tar_ls_));
  const auto& active_joints = active_jnts_extractor_.activeJointNames();
  if (tar_js_conv_.jntArrayToJointStateVel(velocities, active_joints) < 0)
  {
    TOBAS_ERROR("Failed to convert Jntarray to JointState: ", tar_js_conv_.errorMessage());
    return -1;
  }

  // Fill output message
  for (const auto& [name, vel] : tobas_std::zip(tar_js_conv_.getNamesMsg(), tar_js_conv_.getVelocitiesMsg()))
    velocities_msg.commands.emplace_back(name, vel);

  return 0;
}

void VelocityControllerRos::currentJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& cur_js)
{
  cur_js_ = cur_js;

  if (tar_js_ == nullptr && tar_ls_ == nullptr)
    return;

  const auto time_after_last_cmd = (get_clock()->now() - t_last_cmd_).seconds();
  if (is_commanded_ && time_after_last_cmd > tobas::kAutoResetTimeThreshold)
  {
    tar_js_ =std::make_unique<sensor_msgs::msg::JointState>(home_js_);
    tar_ls_ = nullptr;
    is_commanded_ = false;
    TOBAS_WARN(
      "The target joint states are automatically reset because ", tobas::kAutoResetTimeThreshold,
      " seconds have elapsed since the last command.");
  }

  // Create joint velocities command
  const auto velocities_msg =std::make_unique<tobas_msgs::JointCommandArray>();

  // Joint space control or Task space control
  if (tar_js_ != nullptr)
  {
    if (jointSpaceControl(*velocities_msg) < 0)
      return;
  }
  else if (tar_ls_ != nullptr)
  {
    if (taskSpaceControl(*velocities_msg) < 0)
      return;
  }
  else
  {
    TOBAS_ERROR("Both target joint state and target link state are NULL.");
    return;
  }

  // Publish joint velocities command
  velocities_pub_->publish(velocities_msg);
}

void VelocityControllerRos::targetJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& tar_js)
{
  tar_js_ = tar_js;
  tar_ls_ = nullptr;

  t_last_cmd_ = get_clock()->now();
  is_commanded_ = true;
}

void VelocityControllerRos::targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls)
{
  tar_ls_ = tar_ls;
  tar_js_ = nullptr;

  t_last_cmd_ = get_clock()->now();
  is_commanded_ = true;
}

void VelocityControllerRos::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  bool success = true;

  // Joint space control
  if (cfg.joint_time_constant <= 0)
  {
    TOBAS_ERROR("Tracking time constant must be positive.");
    success = false;
  }

  // Task space control
  if (cfg.linear_time_constant <= 0)
  {
    TOBAS_ERROR("Linear time constant must be positive.");
    success = false;
  }
  if (cfg.angular_time_constant <= 0)
  {
    TOBAS_ERROR("Angular time constant must be positive.");
    success = false;
  }

  if (success)
  {
    jnt_time_const_ = cfg.joint_time_constant;
    if (!vel_ctrl_.setLinearTimeConst(cfg.linear_time_constant))
      TOBAS_ERROR("Failed to set linear tracking time constant.");
    if (!vel_ctrl_.setAngularTimeConst(cfg.angular_time_constant))
      TOBAS_ERROR("Failed to set angular tracking time constant.");
    TOBAS_INFO("Dynamic parameters are updated.");
  }
  else
  {
    TOBAS_ERROR("Failed to update dynamic parameters.");
  }
}
}  // namespace tobas_manipulation
