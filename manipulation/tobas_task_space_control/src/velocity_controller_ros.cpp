#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

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
  : super(nh, pnh, name), js_converter_(drone_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  js_converter_.updateInternalDataStructures();
  jntarraynull_ = JntArray::Zero(drone_.tree().getNrOfJoints());

  registerPublishers();
  registerSubscribers();

  check_topics_timer_ =
    nh_.createTimer(ros::Duration(tobas::kCheckTopicsTimerPeriod), &self::checkTopicsTimerCb, this);
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
  if (js->name.size() != js->position.size())
  {
    rosError(name_, "Joint state size mismatch.");
    return;
  }

  js_ = js;
}

void VelocityControllerRos::cartStateCb(const tobas_msgs::CartesianStateConstPtr& cs)
{
  if (cs->name.size() != cs->pose.size())
  {
    rosError(name_, "Cartesian state size mismatch.");
    return;
  }

  if (!is_initialized_)
  {
    if (odom_ != nullptr && js_ != nullptr)
    {
      check_topics_timer_.stop();
      is_initialized_ = true;
    }
    return;
  }

  const auto np = cs->name.size();  // The number of endpoints
  if (
    cs->frame_id.size() != np || cs->pose.size() != np || cs->twist.size() != np
    || cs->wrench.size() != np)
  {
    rosError(name_, "Cartesian state size mismatch.");
    return;
  }

  // デカルト座標系の目標値を更新
  KDL::FrameMap tar_p;
  for (size_t i = 0; i < np; ++i)
  {
    const auto& seg_name = cs->name[i];
    tobas::poseTobasToKDL(cs->pose[i], tar_pi_);
    switch (cs->frame_id[i].data)
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
        rosError(name_, "Unknown frame ID: " << static_cast<int>(cs->frame_id[i].data));
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
  TreeTaskSpaceVelCtrl ctrl(drone_.tree(), cs->name);
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

void VelocityControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (odom_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kOdometryTopic);

  if (js_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kJointStatesTopic)
}
}  // namespace tobas_task_space_control
