#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_msg.h>

#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/operators.hpp>

#include <tobas_msgs/FrameId.h>

#include "../include/tobas_multirotor_controller/velocity_controller_ros.hpp"
#include "../include/tobas_multirotor_controller/constants.hpp"

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace tobas_multirotor_controller
{
VelocityControllerRos::VelocityControllerRos(ros::NodeHandle nh, ros::NodeHandle pnh)
  : super(nh, pnh),
    cmd_level_(tobas_msgs::CommandLevel::NORMAL),
    is_initialized_(false),
    bs_received_(false),
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
  event_sub_ = nh_.subscribe("event", 1, &VelocityControllerRos::eventCb, this);
  base_state_sub_ = nh_.subscribe("base_state", 1, &VelocityControllerRos::baseStateCb, this);
  vel_yaw_sub_ =
    nh_.subscribe("command/velocity_yaw", 1, &VelocityControllerRos::velocityYawCb, this);
}

bool VelocityControllerRos::isReady()
{
  if (!bs_received_)
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
  const auto cur_vel_W = cur_bs_.pose.euler * cur_bs_.twist.vel;
  vel_controller_.update(cur_vel_W, tar_vel_W_, tar_acc_W_);

  // 非線形変換
  acc_controller_.update(
    tar_acc_W_, cur_bs_.pose.euler.yaw, rpy_thrust_.thrust, rpy_thrust_.rpy.roll,
    rpy_thrust_.rpy.pitch);

  // 目標姿勢とスラストを発行
  rpy_thrust_pub_.publish(rpy_thrust_);
}

void VelocityControllerRos::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      ros::shutdown();
      break;
    default:
      break;
  }
}

void VelocityControllerRos::baseStateCb(const tobas_msgs::BaseState& bs)
{
  if (!bs_received_)
  {
    bs_received_ = true;
  }

  cur_bs_ = bs;

  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      initialize();
      is_initialized_ = true;
      rosInfo("Velocity controller is ready.");
    }
    return;
  }

  // トピックが揃っていたら，状態を観測するたびに一回だけ制御器を回す．
  runOnce();
}

void VelocityControllerRos::velocityYawCb(const tobas_msgs::VelocityYaw& vel_yaw)
{
  if (!bs_received_)
  {
    return;
  }

  // コマンドレベルの処理
  if (vel_yaw.level.data < cmd_level_)
  {
    rosErrorThrottle(
      kErrorPeriod, "The command is ignored because its level "
                      << static_cast<int>(vel_yaw.level.data)
                      << "is lower than the current command level " << static_cast<int>(cmd_level_)
                      << ".");
    return;
  }
  if (vel_yaw.level.data > cmd_level_)
  {
    rosInfo(
      "The command level is raised from " << static_cast<int>(cmd_level_) << " to "
                                          << static_cast<int>(vel_yaw.level.data) << ".");
    cmd_level_ = vel_yaw.level.data;
  }

  // 目標速度を更新
  switch (vel_yaw.frame_id.data)
  {
    case tobas_msgs::FrameId::GLOBAL:
    {
      tar_vel_W_ = vel_yaw.vel;
      break;
    }
    case tobas_msgs::FrameId::LOCAL:
    {
      tar_vel_W_ = cur_bs_.pose.euler * vel_yaw.vel;
      break;
    }
    default:
    {
      rosError("Invalid FrameId: " << static_cast<int>(vel_yaw.frame_id.data));
      return;
    }
  }

  // 目標ヨー角を更新 (そのまま流すだけ)
  rpy_thrust_.rpy.yaw = vel_yaw.yaw;

  if (!vel_yaw_received_)
  {
    vel_yaw_received_ = true;
  }
}

void VelocityControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!bs_received_)
    rosWarn("Base state is not received yet.");

  if (!vel_yaw_received_)
    rosInfo("Waiting for Velocity & Yaw command.");
}

void VelocityControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  updateDynamicParams(cfg);
  vel_controller_.configure(dynamic_params_vel_);
  acc_controller_.configure(dynamic_params_acc_);
  rosInfo("Dynamic parameters are updated.");
}
}  // namespace tobas_multirotor_controller
