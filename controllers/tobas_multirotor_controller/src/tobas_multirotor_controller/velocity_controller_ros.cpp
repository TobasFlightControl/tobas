#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_msg.h>

#include <dh_std_tools/vector.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/operators.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_msgs/FrameId.h>

#include "../../include/tobas_multirotor_controller/velocity_controller_ros.hpp"
#include "../../include/tobas_multirotor_controller/constants.hpp"

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace tobas_multirotor_controller
{
VelocityControllerRos::VelocityControllerRos()
  : super(),
    jnt_name_parser_(drone_.tree()),
    is_initialized_(false),
    bs_received_(false),
    js_received_(false),
    cmd_received_(false)
{
  getRosParams();
  drone_.loadFromParam(ns_);

  jnt_name_parser_.updateInternalDataStructures();
  is_transformable_ = drone_.activeJointNames().size() > 0;

  // 各コントローラを初期化
  vel_controller_.reset(new VelocityController(dynamic_params_vel_));
  acc_controller_.reset(new AccelerationController(drone_, gravity_));
  rot_controller_.reset(new RotationController(drone_, dynamic_params_rot_));

  q_.resize(drone_.tree().getNrOfJoints());
  rotor_speeds_.speeds.resize(drone_.numRotors(), 0.);

  registerPublishers();
  registerSubscribers();
  createTimers();

  // Dynamic Reconfigure
  ConfigServer::CallbackType f =
    boost::bind(&VelocityControllerRos::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void VelocityControllerRos::getRosParams()
{
  dh_ros::getParam("/gravity", gravity_);

  // velocity controller
  dh_ros::getParam(kCtrlName + "/natural_frequency", dynamic_params_vel_.natural_freq);
  dh_ros::getParam(kCtrlName + "/damping_ratio", dynamic_params_vel_.damp_ratio);

  // rotation_controller
  dh_ros::getParam(kCtrlName + "/prediction_horizon", dynamic_params_rot_.pred_horizon);
  dh_ros::getParam(kCtrlName + "/prediction_steps", dynamic_params_rot_.pred_steps);
  dh_ros::getParam(kCtrlName + "/rotation_decay", dynamic_params_rot_.rot_decay);
  dh_ros::getParam(kCtrlName + "/angular_velocity_decay", dynamic_params_rot_.angvel_decay);
  dh_ros::getParam(kCtrlName + "/rotation_weight", dynamic_params_rot_.rot_weight);
  dh_ros::getParam(kCtrlName + "/angular_velocity_weight", dynamic_params_rot_.angvel_weight);
  dh_ros::getParam(kCtrlName + "/thrust_force_weight", dynamic_params_rot_.thrust_weight);
  dh_ros::getParam(kCtrlName + "/thrust_force_rate_weight", dynamic_params_rot_.thrust_rate_weight);
}

void VelocityControllerRos::registerPublishers()
{
  rotor_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>("command/motor_speed", 1, false);
}

void VelocityControllerRos::registerSubscribers()
{
  base_state_sub_ = nh_.subscribe("base_state", 1, &VelocityControllerRos::baseStateCb, this);
  if (is_transformable_)
  {
    joint_state_sub_ = nh_.subscribe("joint_states", 1, &VelocityControllerRos::jointStateCb, this);
  }
  cmd_sub_ = nh_.subscribe("command/velocity_yaw", 1, &VelocityControllerRos::commandCb, this);
}

void VelocityControllerRos::createTimers()
{
  check_topics_timer_ = nh_.createTimer(
    ros::Duration(kCheckTopicsTimerPeriod), &VelocityControllerRos::checkTopicsTimerCb, this);
}

bool VelocityControllerRos::isReady()
{
  if (!bs_received_)
  {
    return false;
  }

  if (is_transformable_ && !js_received_)
  {
    return false;
  }

  return true;
}

void VelocityControllerRos::initialize()
{
  tar_rpy_.yaw = cur_bs_.pose.euler.yaw;  // ヨー角は初期状態を目標状態にする
  u_opt_ = VectorXd::Zero(drone_.numRotorsInAxis(Axis::Z_POSITIVE));
}

void VelocityControllerRos::updateDynamicParams(const ConfigType& cfg)
{
  dynamic_params_rot_.pred_horizon = cfg.prediction_horizon;
  dynamic_params_rot_.pred_steps = cfg.prediction_steps;
  dynamic_params_rot_.rot_decay = cfg.rotation_decay;
  dynamic_params_rot_.angvel_decay = cfg.angular_velocity_decay;
  dynamic_params_rot_.rot_weight = cfg.rotation_weight;
  dynamic_params_rot_.angvel_weight = cfg.angular_velocity_weight;
  dynamic_params_rot_.thrust_weight = cfg.thrust_force_weight;
  dynamic_params_rot_.thrust_rate_weight = cfg.thrust_force_rate_weight;
}

void VelocityControllerRos::runOnce()
{
  // 速度制御器
  vel_controller_->update(cur_bs_.twist.vel, tar_vel_W_, tar_acc_W_);

  // 非線形変換
  acc_controller_->update(tar_acc_W_, cur_bs_.pose.euler.yaw, U_, tar_rpy_.roll, tar_rpy_.pitch);
  if (U_ < 0. || acc_controller_->maxU() < U_)
  {
    const double& max_U = acc_controller_->maxU();
    dh_ros::rosWarnThrottle(
      kWarnPeriod, "U_out = " + to_string(U_) + " is out of range [0, " + to_string(max_U) + "].");
    U_ = dh_std::clamp(U_, 0., max_U);
  }

  // 姿勢制御器
  rot_controller_->update(cur_bs_.pose.euler, cur_bs_.twist.rot, q_, U_, tar_rpy_, u_opt_);

  // 各モータの回転速度メッセージを更新
  rotor_speeds_.header.stamp = cur_bs_.header.stamp;
  ctrlInputToRotorSpeeds(u_opt_, rotor_speeds_);

  // モータ速度を発行
  rotor_speeds_pub_.publish(rotor_speeds_);
}

void VelocityControllerRos::ctrlInputToRotorSpeeds(
  const Eigen::VectorXd& u,
  tobas_msgs::RotorSpeeds& speeds)
{
  const auto ver_prop_idxes = drone_.rotorConfigIdxInAxis(Axis::Z_POSITIVE);
  assert(u.rows() == ver_prop_idxes.size());

  for (int i = 0; i < u.rows(); ++i)
  {
    if (u(i) < -1.)
    {
      dh_ros::rosFatal("Negative thrust force: " + to_string(u(i)) + " [N]");
      // TODO: 防御モードに移行
    }

    const auto& idx = ver_prop_idxes[i];
    speeds.speeds[idx] = sqrt(max(u(i), 0.) / drone_.rotorConfig(idx).motor_constant);
  }
}

void VelocityControllerRos::baseStateCb(const StateMsg& bs)
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
      dh_ros::rosInfo("Velocity controller is ready.");
    }
    return;
  }

  // トピックが揃っていたら，状態を観測するたびに一回だけ制御器を回す．
  runOnce();
}

void VelocityControllerRos::jointStateCb(const sensor_msgs::JointState& js)
{
  if (js.name.size() != js.position.size())
  {
    dh_ros::rosError("The size of joint name and position is different.");
    return;
  }

  for (const auto& jnt_name : drone_.activeJointNames())
  {
    try
    {
      const auto msg_idx = dh_std::findIndex(js.name, jnt_name);  // msg内でのインデックス
      const auto& jnt_pos = js.position[msg_idx];
      const auto& kdl_idx = jnt_name_parser_.jointIndex(jnt_name);  // Tree内でのインデックス
      q_(kdl_idx) = jnt_pos;
    }
    catch (const exception& e)
    {
      dh_ros::rosError(e.what());
      return;
    }
  }

  js_received_ = true;
}

void VelocityControllerRos::commandCb(const CmdMsg& cmd)
{
  switch (cmd.frame_id.frame_id)
  {
    case tobas_msgs::FrameId::GLOBAL:
    {
      tar_vel_W_ = cmd.vel;
      tar_rpy_.yaw = cmd.yaw;
      break;
    }
    case tobas_msgs::FrameId::LOCAL:
    {
      tar_vel_W_ = cur_bs_.pose.euler * cmd.vel;
      tar_rpy_.yaw = cmd.yaw;
      break;
    }
    default:
    {
      dh_ros::rosError("Invalid FrameId: " + to_string(cmd.frame_id.frame_id));
      return;
    }
  }

  cmd_received_ = true;
}

void VelocityControllerRos::checkTopicsTimerCb(const ros::TimerEvent& event)
{
  if (!bs_received_)
  {
    dh_ros::rosWarn("Base state is not received yet.");
  }

  if (is_transformable_ && !js_received_)
  {
    dh_ros::rosWarn("Joint states are not received yet.");
  }
}

void VelocityControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t level)
{
  updateDynamicParams(cfg);
  rot_controller_->reconfigure(dynamic_params_rot_);
}
}  // namespace tobas_multirotor_controller
