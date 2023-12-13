#include <dh_std_tools/zip.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/conversions/kdl_msg.hpp>
#include <tobas_msgs/JointVelocities.h>

#include "../include/tobas_task_space_control/velocity_controller_ros.hpp"

using namespace std;
using namespace KDL;

namespace tobas_task_space_control
{
VelocityControllerRos::VelocityControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const std::string& name)
  : super(nh, pnh, name), js_converter_(drone_.tree()), server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  js_converter_.updateInternalDataStructures();
  if (!js_converter_.setJointNames(drone_.postureDefiningJointNames()))
    ROS_THROW_NAMED(name_, "Failed to set joint names to joint converter.");

  jntarraynull_ = JntArray::Zero(drone_.tree().getNrOfJoints());

  registerPublishers();
  registerSubscribers();

  check_topics_timer_ =
    nh_.createTimer(ros::Duration(tobas::kCheckTopicsTimerPeriod), &self::checkTopicsTimerCb, this);

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
  js_sub_ = nh_.subscribe(tobas::kJointStatesTopic, 1, &self::jointStateCb, this, tcpNoDelay());
  cs_sub_ = nh_.subscribe(tobas::kCartStatesCmdTopic, 1, &self::cartStateCb, this, tcpNoDelay());
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

void VelocityControllerRos::jointStateCb(const sensor_msgs::JointStateConstPtr& js)
{
  js_ = js;

  if (!is_initialized_)
  {
    if (odom_ != nullptr && js_ != nullptr)
    {
      check_topics_timer_.stop();
      is_initialized_ = true;
      DH_GOOD("Task space velocity controller is ready.");
    }
    return;
  }

  if (cs_ == nullptr)
    return;

  // デカルト座標系の目標値を更新
  KDL::FrameMap tar_p;
  for (const auto& [seg_name, frame_id, pose] : dh_std::zip(cs_->name, cs_->frame_id, cs_->pose))
  {
    tobas::poseTobasToKDL(pose, tar_pi_);
    switch (frame_id.data)
    {
      case tobas_msgs::FrameId::GLOBAL:
      {
        tobas::poseTobasToKDL(odom_->pose, T_W_B_);
        tar_pi_ = T_W_B_.inverse() * tar_pi_;  // T_B_P = T_B_W * T_W_P
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
    tar_p[seg_name] = tar_pi_;  // Base -> Segment tip
  }

  // JointState -> JntArray
  if (js_converter_.jointStateToJntArray(*js_) < 0)
  {
    rosError(name_, "Failed to convert JointState to Jntarray: " << js_converter_.errorMessage());
    return;
  }
  const auto& cur_q = js_converter_.getPositionsKDL();

  // PIDで関節トルクを計算
  // TODO: 毎周期クラスを初期化するのは無駄なので効率化
  TreeTaskSpaceVelCtrl ctrl(drone_.tree(), cs_->name);
  if (!ctrl.setLinearTimeConst(lin_time_const_))
    rosError(name_, "Failed to set linear tracking time constant.");
  if (!ctrl.setAngularTimeConst(ang_time_const_))
    rosError(name_, "Failed to set angular tracking time constant.");
  if (ctrl.CartToJnt(cur_q, tar_p) < 0)
  {
    rosError(name_, "Cartesian controller failed: " << ctrl.errorMessage());
    return;
  }

  // JntArray -> JointState
  if (js_converter_.jntArrayToJointState(jntarraynull_, ctrl.getVelocities(), jntarraynull_) < 0)
  {
    rosError(name_, "Failed to convert Jntarray to JointState: " << js_converter_.errorMessage());
    return;
  }

  // コマンドを発行
  auto velocities_msg = boost::make_shared<tobas_msgs::JointVelocities>();
  velocities_msg->name = js_converter_.getNamesMsg();
  velocities_msg->data = js_converter_.getVelocitiesMsg();
  velocities_pub_.publish(velocities_msg);
}

void VelocityControllerRos::cartStateCb(const tobas_msgs::CartesianStateConstPtr& cs)
{
  const auto np = cs->name.size();  // The number of endpoints
  if (cs->frame_id.size() != np || cs->pose.size() != np)
  {
    rosError(name_, "Cartesian state size mismatch.");
    return;
  }

  cs_ = cs;
}

void VelocityControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (odom_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kOdometryTopic);

  if (js_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kJointStatesTopic)
}

void VelocityControllerRos::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  bool success = true;

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
    lin_time_const_.fill(cfg.linear_time_constant);
    ang_time_const_.fill(cfg.angular_time_constant);
    rosInfo(name_, "Dynamic parameters are updated.");
  }
  else
  {
    rosError(name_, "Failed to update dynamic parameters.");
  }
}
}  // namespace tobas_task_space_control
