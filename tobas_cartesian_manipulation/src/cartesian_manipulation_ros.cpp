#include <std_msgs/Float64.h>

#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/conversions/kdl_msg.hpp>

#include "../include/tobas_cartesian_manipulation/cartesian_manipulation_ros.hpp"

using namespace std;
using namespace KDL;

namespace tobas_cartesian_manipulation
{
CartesianManipulationRos::CartesianManipulationRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const std::string& name)
  : super(nh, pnh, name), jnt_parser_(drone_.tree()), js_converter_(drone_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  jnt_parser_.updateInternalDataStructures();
  js_converter_.updateInternalDataStructures();

  q_.resize(drone_.tree().getNrOfJoints());

  registerPublishers();
  registerSubscribers();

  check_topics_timer_ =
    nh_.createTimer(ros::Duration(tobas::kCheckTopicsTimerPeriod), &self::checkTopicsTimerCb, this);
}

void CartesianManipulationRos::getRosParams()
{
  dh_ros::getParam(
    pnh_, "max_linear_velocity", max_linvel_, TreeIkSolverPos_Online::kDefaultMaxLinearVelocity,
    dh_ros::NON_NEGATIVE);
  dh_ros::getParam(
    pnh_, "max_angular_velocity", max_angvel_, TreeIkSolverPos_Online::kDefaultMaxAngularVelocity,
    dh_ros::NON_NEGATIVE);
}

void CartesianManipulationRos::registerPublishers()
{
  for (const auto& jnt_name : drone_.postureDefiningJoints())
  {
    const auto topic = jnt_name + "_controller/command";
    cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>(topic, 1));
  }
}

void CartesianManipulationRos::registerSubscribers()
{
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
  js_sub_ = nh_.subscribe(tobas::kJointStatesTopic, 1, &self::jointStateCb, this, tcpNoDelay());
  cs_sub_ = nh_.subscribe(tobas::kCartStatesCmdTopic, 1, &self::cartStateCb, this, tcpNoDelay());
}

void CartesianManipulationRos::eventCb(const tobas_msgs::EventConstPtr& event)
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

void CartesianManipulationRos::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  odom_ = odom;

  if (!is_initialized_)
  {
    if (js_ != nullptr && cs_ != nullptr)
    {
      check_topics_timer_.stop();
      t_last_ = odom->header.stamp;
      is_initialized_ = true;
    }
    return;
  }

  // 時刻を更新
  const auto dt = (odom->header.stamp - t_last_).toSec();
  t_last_ = odom->header.stamp;

  // 現在の関節角を更新
  js_converter_.convert(*js_, q_);

  // デカルト座標系の目標値を更新
  frames_.clear();
  for (size_t i = 0; i < cs_->name.size(); ++i)
  {
    tobas::poseTobasToKDL(cs_->pose[i], frame_);
    switch (cs_->frame_id[i].data)
    {
      case tobas_msgs::FrameId::GLOBAL:
      {
        Frame T_W_B;
        tobas::poseTobasToKDL(odom->pose, T_W_B);
        frame_ = T_W_B.inverse() * frame_;  // T_B_P = T_B_W * T_W_P
      }
      case tobas_msgs::FrameId::LOCAL:
      {
        break;
      }
      default:
      {
        rosError(name_, "Unknown frame ID: " << static_cast<int>(cs_->frame_id[i].data));
        break;
      }
    }
    frames_[cs_->name[i]] = frame_;  // Base -> Tip
  }

  // IKを解く
  // TODO: 毎周期クラスを初期化するのは無駄なので効率化
  TreeIkSolverPos_Online ik_solver(drone_.tree(), cs_->name);
  ik_solver.setMaxLinearVelocity(max_linvel_);
  ik_solver.setMaxAngularVelocity(max_angvel_);
  if (ik_solver.CartToJnt(q_, frames_, dt) < 0)
  {
    rosError(name_, "Inverse kinematics finished with error: " << ik_solver.errorMessage());
    return;
  }
  const auto& q_des = ik_solver.getSolution();

  // 位置コマンドを発行
  for (size_t i = 0; i < drone_.postureDefiningJoints().size(); ++i)
  {
    const auto& jnt_name = drone_.postureDefiningJoints()[i];
    const auto& q_nr = jnt_parser_.jointIndex(jnt_name);

    const auto cmd = boost::make_shared<std_msgs::Float64>();
    cmd->data = q_des(q_nr);
    cmd_pubs_[i].publish(cmd);
  }
}

void CartesianManipulationRos::jointStateCb(const sensor_msgs::JointStateConstPtr& js)
{
  if (js->name.size() != js->position.size())
  {
    rosError(name_, "Joint state size mismatch.");
    return;
  }

  js_ = js;
}

void CartesianManipulationRos::cartStateCb(const tobas_msgs::CartesianStateConstPtr& cs)
{
  if (cs->name.size() != cs->pose.size())
  {
    rosError(name_, "Cartesian state size mismatch.");
    return;
  }

  cs_ = cs;
}

void CartesianManipulationRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (odom_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kOdometryTopic);

  if (js_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kJointStatesTopic)
}
}  // namespace tobas_cartesian_manipulation
