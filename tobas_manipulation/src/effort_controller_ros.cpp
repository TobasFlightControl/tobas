#include <tobas_std_tools/zip.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/conversions/kdl_msg.hpp>

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
  setInitTargetJointStates();

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
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
  cur_js_sub_ =
    nh_.subscribe(tobas::kJointStatesTopic, 1, &self::currentJointStateCb, this, tcpNoDelay());
  tar_js_sub_ = nh_.subscribe(kEffortCtrlJSTopic, 1, &self::targetJointStateCb, this, tcpNoDelay());
  tar_cs_sub_ = nh_.subscribe(kEffortCtrlCSTopic, 1, &self::targetCartStateCb, this, tcpNoDelay());
}

void EffortControllerRos::setInitTargetJointStates()
{
  sensor_msgs::JointState init_tar_js;
  for (const auto& joint : drone_.jointConfigs())
  {
    if (joint.cmd_type != tobas::JointConfig::EFFORT)
      continue;
    init_tar_js.name.push_back(joint.name);
    init_tar_js.position.push_back(joint.home_pos);
    init_tar_js.velocity.push_back(0.);
    init_tar_js.effort.push_back(0.);
  }

  if (init_tar_js.name.size() > 0)
    tar_js_ = boost::make_shared<sensor_msgs::JointState>(init_tar_js);
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
  Frame tar_pi, T_W_B;
  FrameMap tar_p;
  TwistMap tar_v;
  AccelMap a_ff;
  WrenchMap f_ext;
  for (const auto& [seg_name, frame_id, pose, twist, accel, wrench] : tobas_std::zip(
         tar_cs_->name, tar_cs_->frame_id, tar_cs_->pose, tar_cs_->twist, tar_cs_->accel,
         tar_cs_->wrench))
  {
    tobas::poseTobasToKDL(pose, tar_pi);
    auto tar_vi = twist;
    auto ai_ff = accel;
    auto fi_ext = wrench;
    switch (frame_id.data)
    {
      case tobas_msgs::FrameId::GLOBAL:
      {
        if (odom_ == nullptr)
        {
          rosWarnThrottle(
            kOdomNotReceivedWarnPeriod, name_,
            "Since odometry has not been received yet, commands in the global frame is ignored.");
          return -1;
        }

        tobas::poseTobasToKDL(odom_->pose, T_W_B);
        tar_pi = T_W_B.inverse() * tar_pi;  // T_B_P = T_B_W * T_W_P
        tar_vi = T_W_B.M.inverse(tar_vi);
        ai_ff = T_W_B.M.inverse(ai_ff);
        fi_ext = T_W_B.M.inverse(fi_ext);

        break;
      }
      case tobas_msgs::FrameId::LOCAL:
      {
        break;
      }
      default:
      {
        rosError(name_, "Unknown frame ID: " << static_cast<int>(frame_id.data));
        return -1;
      }
    }
    tar_p[seg_name] = tar_pi;  // Base -> Segment tip
    tar_v[seg_name] = tar_vi;
    a_ff[seg_name] = ai_ff;
    f_ext[seg_name] = fi_ext;
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

void EffortControllerRos::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  odom_ = odom;
}

void EffortControllerRos::currentJointStateCb(const sensor_msgs::JointStateConstPtr& cur_js)
{
  cur_js_ = cur_js;

  if (tar_js_ == nullptr && tar_cs_ == nullptr)
    return;

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
}

void EffortControllerRos::targetCartStateCb(const tobas_msgs::CartesianStateConstPtr& tar_cs)
{
  if (odom_ == nullptr)
    return;

  const auto np = tar_cs->name.size();  // The number of endpoints
  if (
    tar_cs->frame_id.size() != np || tar_cs->pose.size() != np || tar_cs->twist.size() != np
    || tar_cs->accel.size() != np || tar_cs->wrench.size() != np)
  {
    rosError(name_, "Cartesian state size mismatch.");
    return;
  }

  tar_cs_ = tar_cs;
  tar_js_ = nullptr;
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
