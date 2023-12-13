#include <dh_std_tools/zip.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/conversions/kdl_msg.hpp>
#include <tobas_msgs/JointVelocities.h>

#include "../include/tobas_manipulation/velocity_controller_ros.hpp"

using namespace std;
using namespace KDL;

namespace tobas_manipulation
{
VelocityControllerRos::VelocityControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const std::string& name)
  : super(nh, pnh, name), cur_js_conv_(drone_.tree()), tar_js_conv_(drone_.tree()), server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  cur_js_conv_.updateInternalDataStructures();
  tar_js_conv_.updateInternalDataStructures();

  if (!cur_js_conv_.setJointNames(drone_.postureDefiningJointNames()))
    ROS_THROW_NAMED(name_, "Failed to set joint names to joint converter.");
  if (!tar_js_conv_.setJointNames(drone_.postureDefiningJointNames()))
    ROS_THROW_NAMED(name_, "Failed to set joint names to joint converter.");

  jntarraynull_ = JntArray::Zero(drone_.tree().getNrOfJoints());

  // Set initial target joint states
  sensor_msgs::JointState init_tar_js;
  for (const auto& joint : drone_.jointConfigs())
  {
    init_tar_js.name.push_back(joint.name);
    init_tar_js.position.push_back(joint.init_pos);
    init_tar_js.velocity.push_back(0.);
    init_tar_js.effort.push_back(0.);
  }
  tar_js_ = boost::make_shared<sensor_msgs::JointState>(init_tar_js);

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
  tar_js_sub_ =
    nh_.subscribe(tobas::kJointStatesCmdTopic, 1, &self::targetJointStateCb, this, tcpNoDelay());
  tar_cs_sub_ =
    nh_.subscribe(tobas::kCartStatesCmdTopic, 1, &self::targetCartStateCb, this, tcpNoDelay());
}

int VelocityControllerRos::jointSpaceControl(JntArray& velocities)
{
  if (tar_js_conv_.jointStateToJntArray(*tar_js_) < 0)
  {
    rosError(
      name_, "Failed to convert target JointState to Jntarray: " << tar_js_conv_.errorMessage());
    return -1;
  }

  // 目標関節速度を計算
  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  const auto& tar_q = cur_js_conv_.getPositionsKDL();
  const auto gain = 1 / jnt_time_const_;
  velocities = gain * (tar_q - cur_q);

  return 0;
}

int VelocityControllerRos::taskSpaceControl(JntArray& velocities)
{
  Frame tar_pi, T_W_B;
  KDL::FrameMap tar_p;
  for (const auto& [seg_name, frame_id, pose] :
       dh_std::zip(tar_cs_->name, tar_cs_->frame_id, tar_cs_->pose))
  {
    tobas::poseTobasToKDL(pose, tar_pi);
    switch (frame_id.data)
    {
      case tobas_msgs::FrameId::GLOBAL:
      {
        tobas::poseTobasToKDL(odom_->pose, T_W_B);
        tar_pi = T_W_B.inverse() * tar_pi;  // T_B_P = T_B_W * T_W_P
      }
      case tobas_msgs::FrameId::LOCAL:
      {
        break;
      }
      default:
      {
        rosError(name_, "Unknown frame ID: " << static_cast<int>(frame_id.data));
        break;
      }
    }
    tar_p[seg_name] = tar_pi;  // Base -> Segment tip
  }

  // 目標関節速度を計算
  // TODO: 毎周期クラスを初期化するのは無駄なので効率化
  TreeTaskSpaceVelCtrl ctrl(drone_.tree(), tar_cs_->name);
  if (!ctrl.setLinearTimeConst(lin_time_const_))
    rosError(name_, "Failed to set linear tracking time constant.");
  if (!ctrl.setAngularTimeConst(ang_time_const_))
    rosError(name_, "Failed to set angular tracking time constant.");

  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  if (ctrl.CartToJnt(cur_q, tar_p) < 0)
  {
    rosError(name_, "Cartesian controller failed: " << ctrl.errorMessage());
    return -1;
  }
  velocities = ctrl.getVelocities();

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

  // JointState -> JntArray
  if (cur_js_conv_.jointStateToJntArray(*cur_js_) < 0)
  {
    rosError(
      name_, "Failed to convert current JointState to Jntarray: " << cur_js_conv_.errorMessage());
    return;
  }

  // Joint space control or Task space control
  JntArray velocities;
  if (tar_js_ != nullptr)
  {
    if (jointSpaceControl(velocities) < 0)
      return;
  }
  else if (tar_cs_ != nullptr)
  {
    if (odom_ == nullptr)
      return;

    if (taskSpaceControl(velocities) < 0)
      return;
  }
  else
  {
    rosError(name_, "Both target joint state and target cartesian state are NULL.");
    return;
  }

  // JntArray -> JointState
  if (cur_js_conv_.jntArrayToJointState(jntarraynull_, velocities, jntarraynull_) < 0)
  {
    rosError(name_, "Failed to convert Jntarray to JointState: " << cur_js_conv_.errorMessage());
    return;
  }

  // コマンドを発行
  auto velocities_msg = boost::make_shared<tobas_msgs::JointVelocities>();
  velocities_msg->name = cur_js_conv_.getNamesMsg();
  velocities_msg->data = cur_js_conv_.getVelocitiesMsg();
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
    lin_time_const_.fill(cfg.linear_time_constant);
    ang_time_const_.fill(cfg.angular_time_constant);
    rosInfo(name_, "Dynamic parameters are updated.");
  }
  else
  {
    rosError(name_, "Failed to update dynamic parameters.");
  }
}
}  // namespace tobas_manipulation
