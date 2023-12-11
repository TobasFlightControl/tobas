#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/conversions/kdl_msg.hpp>
#include <tobas_msgs/JointEfforts.h>

#include "../include/tobas_joint_control/effort_controller_ros.hpp"

using namespace std;
using namespace KDL;

namespace tobas_joint_control
{
EffortControllerRos::EffortControllerRos(
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
  js_sub_ = nh_.subscribe(tobas::kJointStatesTopic, 1, &self::jointStateCb, this, tcpNoDelay());
  cs_sub_ = nh_.subscribe(tobas::kCartStatesCmdTopic, 1, &self::cartStateCb, this, tcpNoDelay());
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

void EffortControllerRos::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  odom_ = odom;
}

void EffortControllerRos::jointStateCb(const sensor_msgs::JointStateConstPtr& js)
{
  if (js->name.size() != js->position.size())
  {
    rosError(name_, "Joint state size mismatch.");
    return;
  }

  js_ = js;
}

void EffortControllerRos::cartStateCb(const tobas_msgs::CartesianStateConstPtr& cs)
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
  KDL::TwistMap tar_v;
  KDL::AccelMap a_ff;
  KDL::WrenchMap f_ext;
  for (size_t i = 0; i < np; ++i)
  {
    const auto& seg_name = cs->name[i];
    tobas::poseTobasToKDL(cs->pose[i], tar_pi_);
    auto tar_vi = cs->twist[i];
    auto ai_ff = cs->accel[i];
    auto fi_ext = cs->wrench[i];
    switch (cs->frame_id[i].data)
    {
      case tobas_msgs::FrameId::GLOBAL:
      {
        tobas::poseTobasToKDL(odom_->pose, T_W_B_);
        tar_pi_ = T_W_B_.inverse() * tar_pi_;  // T_B_P = T_B_W * T_W_P
        tar_vi = T_W_B_.M.inverse(tar_vi);
        ai_ff = T_W_B_.M.inverse(ai_ff);
        fi_ext = T_W_B_.M.inverse(fi_ext);
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
    tar_v[seg_name] = tar_vi;
    a_ff[seg_name] = ai_ff;
    f_ext[seg_name] = fi_ext;
  }

  // JointState -> JntArray
  if (js_converter_.jointStateToJntArray(*js_) < 0)
  {
    rosError(name_, "Failed to convert JointState to Jntarray: " << js_converter_.errorMessage());
    return;
  }
  const auto& cur_q = js_converter_.getPositionsKDL();
  const auto& cur_qd = js_converter_.getVelocitiesKDL();

  // PIDで関節トルクを計算
  // TODO: 毎周期クラスを初期化するのは無駄なので効率化
  TreeTaskSpacePID pid(drone_.tree(), cs->name);
  if (pid.CartToJnt(cur_q, cur_qd, tar_p, tar_v, a_ff, f_ext) < 0)
  {
    rosError(name_, "Cartesian PID failed: " << pid.errorMessage());
    return;
  }

  // JntArray -> JointState
  if (js_converter_.jntArrayToJointState(jntarraynull_, jntarraynull_, pid.getEfforts()) < 0)
  {
    rosError(name_, "Failed to convert Jntarray to JointState: " << js_converter_.errorMessage());
    return;
  }

  // コマンドを発行
  auto efforts_msg = boost::make_shared<tobas_msgs::JointEfforts>();
  efforts_msg->name = js_converter_.getNamesMsg();
  efforts_msg->data = js_converter_.getEffortsMsg();
  efforts_pub_.publish(efforts_msg);
}

void EffortControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (odom_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kOdometryTopic);

  if (js_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kJointStatesTopic)
}
}  // namespace tobas_joint_control
