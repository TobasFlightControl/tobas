#include <tobas_std_tools/zip.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_kdl_msgs/conversion/kdl_msg.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_manipulation/velocity_controller_ros.hpp"
#include "../include/tobas_manipulation/common.hpp"

using namespace std;
using namespace KDL;

namespace tobas_manipulation
{
VelocityControllerRos::VelocityControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    cur_js_conv_(drone_.tree()),
    tar_js_conv_(drone_.tree()),
    active_jnts_extractor_(drone_.tree()),
    vel_ctrl_(drone_.tree()),
    server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  cur_js_conv_.updateInternalDataStructures();
  tar_js_conv_.updateInternalDataStructures();
  active_jnts_extractor_.updateInternalDataStructures();
  vel_ctrl_.updateInternalDataStructures();

  jntarraynull_ = JntArray::Zero(drone_.tree().getNrOfJoints());

  // 速度指令タイプの関節のホームポジションを取得
  for (const auto& joint : drone_.jointConfigs())
  {
    if (joint.cmd_type != tobas::JointConfig::VELOCITY)
      continue;
    home_js_.name.push_back(joint.name);
    home_js_.position.push_back(joint.home_pos);
    home_js_.velocity.push_back(0.);
    home_js_.effort.push_back(0.);
  }

  // ホームポジションを初期目標状態に設定
  if (home_js_.name.size() > 0)
    tar_js_ = boost::make_shared<sensor_msgs::JointState>(home_js_);

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&self::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void VelocityControllerRos::getRosParams()
{
}

void VelocityControllerRos::registerPublishers()
{
  velocities_pub_ = nh_.advertise<tobas_msgs::JointVelocities>(tobas::kJointVelocitiesCmdTopic, 1);
}

void VelocityControllerRos::registerSubscribers()
{
  cur_js_sub_ =
    nh_.subscribe(tobas::kJointStatesTopic, 1, &self::currentJointStateCb, this, tcpNoDelay());
  tar_js_sub_ =
    nh_.subscribe(tobas::kVelCtrlJSTopic, 1, &self::targetJointStateCb, this, tcpNoDelay());
  tar_cs_sub_ =
    nh_.subscribe(tobas::kVelCtrlCSTopic, 1, &self::targetCartStateCb, this, tcpNoDelay());
}

int VelocityControllerRos::jointSpaceControl(tobas_msgs::JointVelocities& velocities_msg)
{
  // JointState -> JntArray
  if (cur_js_conv_.jointStateToJntArray(*cur_js_) < 0)
  {
    rosError(
      name_, "Failed to convert current JointState to Jntarray: " << cur_js_conv_.errorMessage());
    return -1;
  }
  if (tar_js_conv_.jointStateToJntArray(*tar_js_) < 0)
  {
    rosError(
      name_, "Failed to convert target JointState to Jntarray: " << tar_js_conv_.errorMessage());
    return -1;
  }

  // 目標関節速度を計算
  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  const auto& tar_q = tar_js_conv_.getPositionsKDL();
  const auto gain = 1 / jnt_time_const_;
  const auto velocities = gain * (tar_q - cur_q);

  // JntArray -> JointState
  if (tar_js_conv_.jntArrayToJointState(jntarraynull_, velocities, jntarraynull_, tar_js_->name) < 0)
  {
    rosError(name_, "Failed to convert Jntarray to JointState: " << tar_js_conv_.errorMessage());
    return -1;
  }

  // Fill output message
  velocities_msg.name = tar_js_conv_.getNamesMsg();
  velocities_msg.data = tar_js_conv_.getVelocitiesMsg();

  return 0;
}

int VelocityControllerRos::taskSpaceControl(tobas_msgs::JointVelocities& velocities_msg)
{
  // JointState -> JntArray
  if (cur_js_conv_.jointStateToJntArray(*cur_js_) < 0)
  {
    rosError(
      name_, "Failed to convert current JointState to Jntarray: " << cur_js_conv_.errorMessage());
    return -1;
  }

  Frame T_Base_Parent;
  KDL::FrameMap tar_p;
  for (const auto& [seg_name, frame] : tobas_std::zip(tar_cs_->name, tar_cs_->frame))
  {
    if (!tf_listener_.lookupTransform(drone_.tree().getRootName(), tar_cs_->header.frame_id))
    {
      rosError(name_, tf_listener_.getErrorMessage());
      continue;
    }

    transformMsgToKDL(tf_listener_.getTransform().transform, T_Base_Parent);
    tar_p[seg_name] = T_Base_Parent * frame;  // Base -> Segment tip
  }

  // 目標関節速度を計算
  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  if (vel_ctrl_.CartToJnt(cur_q, tar_p) < 0)
  {
    rosError(name_, "Cartesian controller failed: " << vel_ctrl_.errorMessage());
    return -1;
  }
  const auto& velocities = vel_ctrl_.getVelocities();

  // JntArray -> JointState
  active_jnts_extractor_.solve(tar_cs_->name);
  const auto& active_joints = active_jnts_extractor_.activeJointNames();
  if (tar_js_conv_.jntArrayToJointState(jntarraynull_, velocities, jntarraynull_, active_joints) < 0)
  {
    rosError(name_, "Failed to convert Jntarray to JointState: " << tar_js_conv_.errorMessage());
    return -1;
  }

  // Fill output message
  velocities_msg.name = tar_js_conv_.getNamesMsg();
  velocities_msg.data = tar_js_conv_.getVelocitiesMsg();

  return 0;
}

void VelocityControllerRos::currentJointStateCb(const sensor_msgs::JointStateConstPtr& cur_js)
{
  cur_js_ = cur_js;

  if (tar_js_ == nullptr && tar_cs_ == nullptr)
    return;

  const auto time_after_last_cmd = (ros::Time::now() - t_last_cmd_).toSec();
  if (is_commanded_ && time_after_last_cmd > tobas::kAutoResetTimeThreshold)
  {
    tar_js_ = boost::make_shared<sensor_msgs::JointState>(home_js_);
    tar_cs_ = nullptr;
    is_commanded_ = false;
    rosWarn(
      name_, "The target joint states are automatically reset because "
               << tobas::kAutoResetTimeThreshold
               << " seconds have elapsed since the last command.");
  }

  // Create joint velocities command
  const auto velocities_msg = boost::make_shared<tobas_msgs::JointVelocities>();

  // Joint space control or Task space control
  if (tar_js_ != nullptr)
  {
    if (jointSpaceControl(*velocities_msg) < 0)
      return;
  }
  else if (tar_cs_ != nullptr)
  {
    if (taskSpaceControl(*velocities_msg) < 0)
      return;
  }
  else
  {
    rosError(name_, "Both target joint state and target cartesian state are NULL.");
    return;
  }

  // Publish joint velocities command
  velocities_pub_.publish(velocities_msg);
}

void VelocityControllerRos::targetJointStateCb(const sensor_msgs::JointStateConstPtr& tar_js)
{
  tar_js_ = tar_js;
  tar_cs_ = nullptr;

  t_last_cmd_ = ros::Time::now();
  is_commanded_ = true;
}

void VelocityControllerRos::targetCartStateCb(const tobas_msgs::CartesianStateConstPtr& tar_cs)
{
  if (tar_cs->name.size() != tar_cs->frame.size())
  {
    rosError(name_, "Cartesian state size mismatch.");
    return;
  }

  tar_cs_ = tar_cs;
  tar_js_ = nullptr;

  t_last_cmd_ = ros::Time::now();
  is_commanded_ = true;
}

void VelocityControllerRos::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  bool success = true;

  // Joint space control
  if (cfg.joint_time_constant <= 0)
  {
    rosError(name_, "Tracking time constant must be positive.");
    success = false;
  }

  // Task space control
  if (cfg.linear_time_constant <= 0)
  {
    rosError(name_, "Linear time constant must be positive.");
    success = false;
  }
  if (cfg.angular_time_constant <= 0)
  {
    rosError(name_, "Angular time constant must be positive.");
    success = false;
  }

  if (success)
  {
    jnt_time_const_ = cfg.joint_time_constant;
    if (!vel_ctrl_.setLinearTimeConst(cfg.linear_time_constant))
      rosError(name_, "Failed to set linear tracking time constant.");
    if (!vel_ctrl_.setAngularTimeConst(cfg.angular_time_constant))
      rosError(name_, "Failed to set angular tracking time constant.");
    rosInfo(name_, "Dynamic parameters are updated.");
  }
  else
  {
    rosError(name_, "Failed to update dynamic parameters.");
  }
}
}  // namespace tobas_manipulation
