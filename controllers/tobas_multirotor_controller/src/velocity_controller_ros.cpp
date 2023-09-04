#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_msg.h>

#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/operators.hpp>

#include <tobas_msgs/RollPitchYawThrust.h>

#include "../include/tobas_multirotor_controller/velocity_controller_ros.hpp"
#include "../include/tobas_multirotor_controller/constants.hpp"

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace tobas_multirotor_controller
{
VelocityControllerRos::VelocityControllerRos(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name),
    cmd_level_(tobas_msgs::CommandLevel::NORMAL),
    is_initialized_(false),
    pt_received_(false),
    vel_yaw_received_(false),
    check_topics_timer_(
      nh_,
      kCheckTopicsTimerPeriod,
      &VelocityControllerRos::checkTopicsTimerCb,
      this),
    server_(ros::NodeHandle(kCtrlName))
{
  getRosParams();

  vel_controller_.configure(dynamic_params_vel_);
  acc_controller_.configure(dynamic_params_acc_);

  registerPublishers();
  registerSubscribers();

  // Dynamic Reconfigure
  ConfigServer::CallbackType f =
    boost::bind(&VelocityControllerRos::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void VelocityControllerRos::getRosParams()
{
  dh_ros::getParam(
    nh_, kCtrlName + "/horizontal_natural_frequency", dynamic_params_vel_.hor_natural_freq);
  dh_ros::getParam(
    nh_, kCtrlName + "/horizontal_damping_ratio", dynamic_params_vel_.hor_damp_ratio);
  dh_ros::getParam(
    nh_, kCtrlName + "/vertical_natural_frequency", dynamic_params_vel_.ver_natural_freq);
  dh_ros::getParam(nh_, kCtrlName + "/vertical_damping_ratio", dynamic_params_vel_.ver_damp_ratio);
  dh_ros::getParam(nh_, kCtrlName + "/max_horizontal_velocity", dynamic_params_vel_.max_hor_vel);
  dh_ros::getParam(nh_, kCtrlName + "/max_vertical_velocity", dynamic_params_vel_.max_ver_vel);

  dh_ros::getParam(nh_, kCtrlName + "/max_horizontal_accel", dynamic_params_acc_.max_hor_acc);
  dh_ros::getParam(nh_, kCtrlName + "/max_vertical_accel", dynamic_params_acc_.max_ver_acc);
}

void VelocityControllerRos::registerPublishers()
{
  rpy_thrust_pub_ = nh_.advertise<tobas_msgs::RollPitchYawThrust>("command/rpy_thrust", 1);
}

void VelocityControllerRos::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &VelocityControllerRos::eventCb, this, tcpNoDelay());
  pt_sub_ = nh_.subscribe("pose_twist", 1, &VelocityControllerRos::poseTwistCb, this, tcpNoDelay());
  vel_yaw_sub_ = nh_.subscribe(
    "command/velocity_yaw", 1, &VelocityControllerRos::velocityYawCb, this, tcpNoDelay());
}

bool VelocityControllerRos::isReady()
{
  if (!pt_received_)
    return false;

  if (!vel_yaw_received_)
    return false;

  return true;
}

void VelocityControllerRos::initialize()
{
}

void VelocityControllerRos::updateDynamicParams(const ConfigType& cfg)
{
  dynamic_params_vel_.hor_natural_freq = cfg.horizontal_natural_frequency;
  dynamic_params_vel_.hor_damp_ratio = cfg.horizontal_damping_ratio;
  dynamic_params_vel_.ver_natural_freq = cfg.vertical_natural_frequency;
  dynamic_params_vel_.ver_damp_ratio = cfg.vertical_damping_ratio;
  dynamic_params_vel_.max_hor_vel = cfg.max_horizontal_velocity;
  dynamic_params_vel_.max_ver_vel = cfg.max_vertical_velocity;

  dynamic_params_acc_.max_hor_acc = cfg.max_horizontal_accel;
  dynamic_params_acc_.max_ver_acc = cfg.max_vertical_accel;
}

void VelocityControllerRos::runOnce()
{
  // 速度制御器
  const auto cur_vel_W = cur_pt_->pose.euler * cur_pt_->twist.vel;
  vel_controller_.update(cur_vel_W, tar_vel_W_, tar_acc_W_);

  // 非線形変換
  const auto rpy_thrust = boost::make_shared<tobas_msgs::RollPitchYawThrust>();
  acc_controller_.update(
    tar_acc_W_, cur_pt_->pose.euler.yaw, rpy_thrust->thrust, rpy_thrust->rpy.roll,
    rpy_thrust->rpy.pitch);

  // 目標ヨー各を更新
  rpy_thrust->rpy.yaw = tar_yaw_;

  // 目標姿勢とスラストを発行
  rpy_thrust_pub_.publish(rpy_thrust);
}

void VelocityControllerRos::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void VelocityControllerRos::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  if (!pt_received_)
  {
    pt_received_ = true;
  }

  cur_pt_ = pt;

  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      initialize();
      is_initialized_ = true;
      rosInfo(name_, "Velocity controller is ready.");
    }
    return;
  }

  // トピックが揃っていたら，状態を観測するたびに一回だけ制御器を回す．
  runOnce();
}

void VelocityControllerRos::velocityYawCb(const tobas_msgs::VelocityYawConstPtr& vel_yaw)
{
  if (!pt_received_)
  {
    return;
  }

  // コマンドレベルの処理
  if (vel_yaw->level.data < cmd_level_)
  {
    rosErrorThrottle(
      kErrorPeriod, name_,
      "The command is ignored because its level " << static_cast<int>(vel_yaw->level.data)
                                                  << "is lower than the current command level "
                                                  << static_cast<int>(cmd_level_) << ".");
    return;
  }
  if (vel_yaw->level.data > cmd_level_)
  {
    rosInfo(
      name_, "The command level is raised from " << static_cast<int>(cmd_level_) << " to "
                                                 << static_cast<int>(vel_yaw->level.data) << ".");
    cmd_level_ = vel_yaw->level.data;
  }

  // 目標速度を更新
  switch (vel_yaw->frame_id.data)
  {
    case tobas_msgs::FrameId::GLOBAL:
    {
      tar_vel_W_ = vel_yaw->vel;
      break;
    }
    case tobas_msgs::FrameId::LOCAL:
    {
      tar_vel_W_ = cur_pt_->pose.euler * vel_yaw->vel;
      break;
    }
    default:
    {
      rosError(name_, "Invalid FrameId: " << static_cast<int>(vel_yaw->frame_id.data));
      return;
    }
  }

  // 目標ヨー角を更新 (そのまま流すだけ)
  tar_yaw_ = vel_yaw->yaw;

  if (!vel_yaw_received_)
  {
    vel_yaw_received_ = true;
  }
}

void VelocityControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!pt_received_)
    rosWarn(name_, "Pose & Twist is not received yet.");

  if (!vel_yaw_received_)
    rosInfo(name_, "Waiting for Velocity & Yaw command.");
}

void VelocityControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  updateDynamicParams(cfg);
  vel_controller_.configure(dynamic_params_vel_);
  acc_controller_.configure(dynamic_params_acc_);
  rosInfo(name_, "Dynamic parameters are updated.");
}
}  // namespace tobas_multirotor_controller
