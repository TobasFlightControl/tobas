#include <tobas_std_tools/zip.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_kdl_msgs/conversion/kdl_msg.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_manipulation/effort_controller_ros.hpp"
#include "../include/tobas_manipulation/common.hpp"

using namespace std;
using namespace KDL;

namespace tobas_manipulation
{
EffortControllerRos::EffortControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    cur_js_conv_(drone_.tree()),
    tar_js_conv_(drone_.tree()),
    active_jnts_extractor_(drone_.tree()),
    pid_js_(drone_.tree()),
    pid_ts_(drone_.tree()),
    server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  cur_js_conv_.updateInternalDataStructures();
  tar_js_conv_.updateInternalDataStructures();
  active_jnts_extractor_.updateInternalDataStructures();
  pid_js_.updateInternalDataStructures();
  pid_ts_.updateInternalDataStructures();

  jntarraynull_ = JntArray::Zero(drone_.tree().getNrOfJoints());

  // 力指令タイプの関節のホームポジションを取得
  for (const auto& joint : drone_.jointConfigs())
  {
    if (joint.cmd_type != tobas::JointConfig::EFFORT)
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

void EffortControllerRos::getRosParams()
{
}

void EffortControllerRos::registerPublishers()
{
  efforts_pub_ = nh_.advertise<tobas_msgs::JointEfforts>(tobas::kJointEffortsCmdTopic, 1);
}

void EffortControllerRos::registerSubscribers()
{
  cur_js_sub_ =
    nh_.subscribe(tobas::kJointStatesTopic, 1, &self::currentJointStateCb, this, tcpNoDelay());
  tar_js_sub_ = nh_.subscribe(kEffortCtrlJSTopic, 1, &self::targetJointStateCb, this, tcpNoDelay());
  tar_cs_sub_ = nh_.subscribe(kEffortCtrlCSTopic, 1, &self::targetCartStateCb, this, tcpNoDelay());
}

int EffortControllerRos::jointSpaceControl(tobas_msgs::JointEfforts& efforts_msg)
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

  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  const auto& cur_qd = cur_js_conv_.getVelocitiesKDL();
  const auto& tar_q = tar_js_conv_.getPositionsKDL();
  const auto& tar_qd = tar_js_conv_.getVelocitiesKDL();

  // PIDで関節トルクを計算
  if (pid_js_.CartToJnt(cur_q, cur_qd, tar_q, tar_qd, jntarraynull_) < 0)
  {
    rosError(name_, "Joint space PID failed: " << pid_js_.errorMessage());
    return -1;
  }
  const auto efforts = tar_js_conv_.getEffortsKDL() + pid_js_.getEfforts();  // FF + FB

  // JntArray -> JointState
  if (tar_js_conv_.jntArrayToJointState(jntarraynull_, jntarraynull_, efforts, tar_js_->name) < 0)
  {
    rosError(name_, "Failed to convert Jntarray to JointState: " << tar_js_conv_.errorMessage());
    return -1;
  }

  // Fill output message
  efforts_msg.name = tar_js_conv_.getNamesMsg();
  efforts_msg.data = tar_js_conv_.getEffortsMsg();

  return 0;
}

int EffortControllerRos::taskSpaceControl(tobas_msgs::JointEfforts& efforts_msg)
{
  // JointState -> JntArray
  if (cur_js_conv_.jointStateToJntArray(*cur_js_) < 0)
  {
    rosError(
      name_, "Failed to convert current JointState to Jntarray: " << cur_js_conv_.errorMessage());
    return -1;
  }

  // デカルト座標系の目標値を更新
  Frame T_Base_Parent;
  FrameMap tar_p;
  TwistMap tar_v;
  AccelMap a_ff;
  WrenchMap f_ext;
  for (const auto& [seg_name, frame, twist, accel, wrench] : tobas_std::zip(
         tar_cs_->name, tar_cs_->frame, tar_cs_->twist, tar_cs_->accel, tar_cs_->wrench))
  {
    if (!tf_listener_.lookupTransform(drone_.tree().getRootName(), tar_cs_->header.frame_id))
    {
      rosError(name_, tf_listener_.getErrorMessage());
      continue;
    }

    // 親フレームで表現された値をベースリンクで表現された値に変換
    transformMsgToKDL(tf_listener_.getTransform().transform, T_Base_Parent);
    tar_p[seg_name] = T_Base_Parent * frame;
    tar_v[seg_name] = T_Base_Parent.M * twist;
    a_ff[seg_name] = T_Base_Parent.M * accel;
    f_ext[seg_name] = T_Base_Parent.M * wrench;
  }

  // PIDで関節トルクを計算
  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  const auto& cur_qd = cur_js_conv_.getVelocitiesKDL();
  if (pid_ts_.CartToJnt(cur_q, cur_qd, tar_p, tar_v, a_ff, f_ext) < 0)
  {
    rosError(name_, "Cartesian PID failed: " << pid_ts_.errorMessage());
    return -1;
  }
  const auto& efforts = pid_ts_.getEfforts();

  // JntArray -> JointState
  active_jnts_extractor_.solve(tar_cs_->name);
  const auto& active_joints = active_jnts_extractor_.activeJointNames();
  if (tar_js_conv_.jntArrayToJointState(jntarraynull_, jntarraynull_, efforts, active_joints) < 0)
  {
    rosError(name_, "Failed to convert Jntarray to JointState: " << tar_js_conv_.errorMessage());
    return -1;
  }

  // Fill output message
  efforts_msg.name = tar_js_conv_.getNamesMsg();
  efforts_msg.data = tar_js_conv_.getEffortsMsg();

  return 0;
}

void EffortControllerRos::currentJointStateCb(const sensor_msgs::JointStateConstPtr& cur_js)
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

  // Create joint efforts command
  const auto efforts_msg = boost::make_shared<tobas_msgs::JointEfforts>();

  // Joint space control or Task space control
  if (tar_js_ != nullptr)
  {
    if (jointSpaceControl(*efforts_msg) < 0)
      return;
  }
  else if (tar_cs_ != nullptr)
  {
    if (taskSpaceControl(*efforts_msg) < 0)
      return;
  }
  else
  {
    rosError(name_, "Both target joint state and target cartesian state are NULL.");
    return;
  }

  // Publish joint efforts command
  efforts_pub_.publish(efforts_msg);
}

void EffortControllerRos::targetJointStateCb(const sensor_msgs::JointStateConstPtr& tar_js)
{
  tar_js_ = tar_js;
  tar_cs_ = nullptr;

  t_last_cmd_ = ros::Time::now();
  is_commanded_ = true;
}

void EffortControllerRos::targetCartStateCb(const tobas_msgs::CartesianStateConstPtr& tar_cs)
{
  const auto np = tar_cs->name.size();  // The number of endpoints
  if (
    tar_cs->frame.size() != np || tar_cs->twist.size() != np || tar_cs->accel.size() != np
    || tar_cs->wrench.size() != np)
  {
    rosError(name_, "Cartesian state size mismatch.");
    return;
  }

  tar_cs_ = tar_cs;
  tar_js_ = nullptr;

  t_last_cmd_ = ros::Time::now();
  is_commanded_ = true;
}

void EffortControllerRos::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  // Joint space control
  if (!pid_js_.setStiffness(cfg.joint_stiffness))
    rosError(name_, "Failed to set joint stiffness.");

  if (!pid_js_.setDamping(cfg.joint_damping))
    rosError(name_, "Failed to set joint damping.");

  // Task space control
  if (!pid_ts_.setLinearStiffness(cfg.linear_stiffness))
    rosError(name_, "Failed to set linear stiffness.");

  if (!pid_ts_.setAngularStiffness(cfg.angular_stiffness))
    rosError(name_, "Failed to set angular stiffness.");

  if (!pid_ts_.setLinearDamping(cfg.linear_damping))
    rosError(name_, "Failed to set linear damping.");

  if (!pid_ts_.setAngularDamping(cfg.angular_damping))
    rosError(name_, "Failed to set angular damping.");
}
}  // namespace tobas_manipulation
