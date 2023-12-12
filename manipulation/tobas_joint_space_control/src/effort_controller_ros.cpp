#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/JointEfforts.h>

#include "../include/tobas_joint_space_control/effort_controller_ros.hpp"

using namespace std;
using namespace KDL;

namespace tobas_joint_space_control
{
EffortControllerRos::EffortControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const std::string& name)
  : super(nh, pnh, name),
    cur_js_conv_(drone_),
    tar_js_conv_(drone_),
    pid_(drone_.tree()),
    server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  cur_js_conv_.updateInternalDataStructures();
  tar_js_conv_.updateInternalDataStructures();
  pid_.updateInternalDataStructures();
  jntarraynull_ = JntArray::Zero(drone_.tree().getNrOfJoints());

  registerPublishers();
  registerSubscribers();

  check_topics_timer_ =
    nh_.createTimer(ros::Duration(tobas::kCheckTopicsTimerPeriod), &self::checkTopicsTimerCb, this);

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
  tar_js_sub_ =
    nh_.subscribe(tobas::kJointStatesCmdTopic, 1, &self::targetJointStateCb, this, tcpNoDelay());
}

void EffortControllerRos::eventCb(const tobas_msgs::EventConstPtr& event)
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

void EffortControllerRos::currentJointStateCb(const sensor_msgs::JointStateConstPtr& cur_js)
{
  cur_js_ = cur_js;

  if (!is_initialized_)
  {
    check_topics_timer_.stop();
    is_initialized_ = true;
    DH_GOOD("Joint space effort controller is ready.");
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

  const auto& cur_q = cur_js_conv_.getPositionsKDL();
  const auto& cur_qd = cur_js_conv_.getVelocitiesKDL();
  const auto& tar_q = tar_js_conv_.getPositionsKDL();
  const auto& tar_qd = tar_js_conv_.getVelocitiesKDL();

  // PIDで関節トルクを計算
  if (pid_.CartToJnt(cur_q, cur_qd, tar_q, tar_qd, jntarraynull_) < 0)
  {
    rosError(name_, "Joint space PID failed: " << pid_.errorMessage());
    return;
  }
  const auto efforts = tar_js_conv_.getEffortsKDL() + pid_.getEfforts();  // FF + FB

  // JntArray -> JointState
  if (cur_js_conv_.jntArrayToJointState(jntarraynull_, jntarraynull_, efforts) < 0)
  {
    rosError(name_, "Failed to convert Jntarray to JointState: " << cur_js_conv_.errorMessage());
    return;
  }

  // コマンドを発行
  auto efforts_msg = boost::make_shared<tobas_msgs::JointEfforts>();
  efforts_msg->name = cur_js_conv_.getNamesMsg();
  efforts_msg->data = cur_js_conv_.getEffortsMsg();
  efforts_pub_.publish(efforts_msg);
}

void EffortControllerRos::targetJointStateCb(const sensor_msgs::JointStateConstPtr& tar_js)
{
  tar_js_ = tar_js;
}

void EffortControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (cur_js_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kJointStatesTopic)
}

void EffortControllerRos::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  bool success = true;

  if (!pid_.setStiffness(cfg.stiffness))
  {
    rosError(name_, "Failed to set stiffness.");
    success = false;
  }

  if (!pid_.setDamping(cfg.damping))
  {
    rosError(name_, "Failed to set damping.");
    success = false;
  }

  if (success)
    rosInfo(name_, "Dynamic parameters are updated.");
}
}  // namespace tobas_joint_space_control
