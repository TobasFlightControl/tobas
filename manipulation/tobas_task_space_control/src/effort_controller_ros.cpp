#include <dh_std_tools/zip.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/conversions/kdl_msg.hpp>
#include <tobas_msgs/JointEfforts.h>

#include "../include/tobas_task_space_control/effort_controller_ros.hpp"

using namespace std;
using namespace KDL;

namespace tobas_task_space_control
{
EffortControllerRos::EffortControllerRos(
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
  js_ = js;

  if (!is_initialized_)
  {
    if (odom_ != nullptr && js_ != nullptr)
    {
      check_topics_timer_.stop();
      is_initialized_ = true;
      DH_GOOD("Task space effort controller is ready.");
    }
  }

  if (cs_ == nullptr)
    return;

  // デカルト座標系の目標値を更新
  KDL::FrameMap tar_p;
  KDL::TwistMap tar_v;
  KDL::AccelMap a_ff;
  KDL::WrenchMap f_ext;
  for (const auto& [seg_name, frame_id, pose, twist, accel, wrench] :
       dh_std::zip(cs_->name, cs_->frame_id, cs_->pose, cs_->twist, cs_->accel, cs_->wrench))
  {
    tobas::poseTobasToKDL(pose, tar_pi_);
    auto tar_vi = twist;
    auto ai_ff = accel;
    auto fi_ext = wrench;
    switch (frame_id.data)
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
        rosError(name_, "Unknown frame ID: " << static_cast<int>(frame_id.data));
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
  TreeTaskSpacePID pid(drone_.tree(), cs_->name);
  if (!pid.setLinearStiffness(lin_kp_))
    rosError(name_, "Failed to set linear stiffness.");
  if (!pid.setAngularStiffness(ang_kp_))
    rosError(name_, "Failed to set angular stiffness.");
  if (!pid.setLinearDamping(lin_kd_))
    rosError(name_, "Failed to set linear damping.");
  if (!pid.setAngularDamping(ang_kd_))
    rosError(name_, "Failed to set angular damping.");
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

void EffortControllerRos::cartStateCb(const tobas_msgs::CartesianStateConstPtr& cs)
{
  const auto np = cs->name.size();  // The number of endpoints
  if (
    cs->frame_id.size() != np || cs->pose.size() != np || cs->twist.size() != np
    || cs->accel.size() != np || cs->wrench.size() != np)
  {
    rosError(name_, "Cartesian state size mismatch.");
    return;
  }

  cs_ = cs;
}

void EffortControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (odom_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kOdometryTopic);

  if (js_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kJointStatesTopic)
}

void EffortControllerRos::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  bool success = true;

  if (cfg.linear_stiffness < 0)
  {
    rosError(name_, "Linear stiffness must be non negative.");
    success = false;
  }
  if (cfg.angular_stiffness < 0)
  {
    rosError(name_, "Angular stiffness must be non negative.");
    success = false;
  }
  if (cfg.linear_damping < 0)
  {
    rosError(name_, "Linear damping must be non negative.");
    success = false;
  }
  if (cfg.angular_damping < 0)
  {
    rosError(name_, "Angular damping must be non negative.");
    success = false;
  }

  if (success)
  {
    lin_kp_.fill(cfg.linear_stiffness);
    ang_kp_.fill(cfg.angular_stiffness);
    lin_kd_.fill(cfg.linear_damping);
    ang_kd_.fill(cfg.angular_damping);
    rosInfo(name_, "Dynamic parameters are updated.");
  }
  else
  {
    rosError(name_, "Failed to update dynamic parameters.");
  }
}
}  // namespace tobas_task_space_control
