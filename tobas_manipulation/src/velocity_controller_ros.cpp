#include <tobas_std_tools/zip.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/conversions/kdl_msg.hpp>

#include "../include/tobas_manipulation/velocity_controller_ros.hpp"
#include "../include/tobas_manipulation/common.hpp"

using namespace std;
using namespace KDL;

namespace tobas_manipulation
{
VelocityControllerRos::VelocityControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const std::string& name)
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
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
  cur_js_sub_ =
    nh_.subscribe(tobas::kJointStatesTopic, 1, &self::currentJointStateCb, this, tcpNoDelay());
  tar_js_sub_ = nh_.subscribe(kVelCtrlJSTopic, 1, &self::targetJointStateCb, this, tcpNoDelay());
  tar_cs_sub_ = nh_.subscribe(kVelCtrlCSTopic, 1, &self::targetCartStateCb, this, tcpNoDelay());
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

  Frame tar_pi, T_W_B;
  KDL::FrameMap tar_p;
  for (const auto& [seg_name, frame_id, pose] :
       tobas_std::zip(tar_cs_->name, tar_cs_->frame_id, tar_cs_->pose))
  {
    tobas::poseTobasToKDL(pose, tar_pi);
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

void VelocityControllerRos::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void VelocityControllerRos::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  odom_ = odom;
}

void VelocityControllerRos::currentJointStateCb(const sensor_msgs::JointStateConstPtr& cur_js)
{
  cur_js_ = cur_js;

  if (tar_js_ == nullptr && tar_cs_ == nullptr)
    return;

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
}

void VelocityControllerRos::targetCartStateCb(const tobas_msgs::CartesianStateConstPtr& tar_cs)
{
  const auto np = tar_cs->name.size();  // The number of endpoints
  if (tar_cs->frame_id.size() != np || tar_cs->pose.size() != np)
  {
    rosError(name_, "Cartesian state size mismatch.");
    return;
  }

  tar_cs_ = tar_cs;
  tar_js_ = nullptr;
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
