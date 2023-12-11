#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/JointVelocities.h>

#include "../include/tobas_joint_control/velocity_controller_ros.hpp"

using namespace std;
using namespace KDL;

namespace tobas_joint_control
{
VelocityControllerRos::VelocityControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const std::string& name)
  : super(nh, pnh, name), cur_js_conv_(drone_), tar_js_conv_(drone_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  cur_js_conv_.updateInternalDataStructures();
  tar_js_conv_.updateInternalDataStructures();
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
  cur_js_sub_ =
    nh_.subscribe(tobas::kJointStatesTopic, 1, &self::currentJointStateCb, this, tcpNoDelay());
  tar_js_sub_ =
    nh_.subscribe(tobas::kJointStatesCmdTopic, 1, &self::targetJointStateCb, this, tcpNoDelay());
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

void VelocityControllerRos::currentJointStateCb(const sensor_msgs::JointStateConstPtr& cur_js)
{
  cur_js_ = cur_js;

  if (!is_initialized_)
  {
    check_topics_timer_.stop();
    is_initialized_ = true;
    DH_GOOD("Joint controller is ready.");
  }

  if (tar_js_ == nullptr)
    return;

  // JointState -> JntArray
  if (cur_js_conv_.jointStateToJntArray(*cur_js_) < 0)
  {
    rosError(
      name_, "Failed to convert current JointState to Jntarray: " << cur_js_conv_.errorMessage());
    return;
  }
  if (tar_js_conv_.jointStateToJntArray(*tar_js_) < 0)
  {
    rosError(
      name_, "Failed to convert target JointState to Jntarray: " << tar_js_conv_.errorMessage());
    return;
  }

  // 目標関節速度を計算
  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  const auto& tar_q = cur_js_conv_.getPositionsKDL();
  const auto tar_qd = gain_ * (tar_q - cur_q);

  // JntArray -> JointState
  if (cur_js_conv_.jntArrayToJointState(jntarraynull_, tar_qd, jntarraynull_) < 0)
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
}

void VelocityControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (cur_js_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kJointStatesTopic)
}
}  // namespace tobas_joint_control
