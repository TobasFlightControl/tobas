#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_msg.h>

#include <dh_std_tools/vector.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/operators.hpp>

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
    z_rotors_(drone_, tobas::Axis::Z_POSITIVE),
    cmd_level_(tobas_msgs::CommandLevel::NORMAL),
    is_initialized_(false),
    bs_received_(false),
    battery_received_(false),
    js_received_(false),
    cmd_received_(false),
    check_topics_timer_(
      nh_,
      kCheckTopicsTimerPeriod,
      &VelocityControllerRos::checkTopicsTimerCb,
      this),
    server_(ros::NodeHandle(kCtrlName))
{
  getRosParams();
  drone_.loadFromParam(ns_);

  jnt_name_parser_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();

  is_transformable_ = drone_.activeJointNames().size() > 0;

  // 各コントローラを初期化
  vel_controller_.reset(new VelocityController(dynamic_params_vel_));
  acc_controller_.reset(new AccelerationController(drone_));
  rot_controller_.reset(new RotationController(drone_, dynamic_params_rot_));

  q_.resize(drone_.tree().getNrOfJoints());
  rotor_speeds_.speeds.resize(drone_.numRotors(), 0.);

  registerPublishers();
  registerSubscribers();

  // Dynamic Reconfigure
  ConfigServer::CallbackType f =
    boost::bind(&VelocityControllerRos::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void VelocityControllerRos::getRosParams()
{
  // velocity controller
  dh_ros::getParam(kCtrlName + "/natural_frequency", dynamic_params_vel_.natural_freq);
  dh_ros::getParam(kCtrlName + "/damping_ratio", dynamic_params_vel_.damp_ratio);

  // rotation_controller
  dh_ros::getParam(kCtrlName + "/prediction_horizon", dynamic_params_rot_.pred_horizon);
  dh_ros::getParam(kCtrlName + "/prediction_steps", dynamic_params_rot_.pred_steps);
  dh_ros::getParam(kCtrlName + "/attitude_decay", dynamic_params_rot_.attitude_decay);
  dh_ros::getParam(kCtrlName + "/heading_decay", dynamic_params_rot_.heading_decay);
  dh_ros::getParam(kCtrlName + "/angular_velocity_decay", dynamic_params_rot_.angvel_decay);
  dh_ros::getParam(kCtrlName + "/attitude_weight", dynamic_params_rot_.attitude_weight);
  dh_ros::getParam(kCtrlName + "/heading_weight", dynamic_params_rot_.heading_weight);
  dh_ros::getParam(kCtrlName + "/angular_velocity_weight", dynamic_params_rot_.angvel_weight);
  dh_ros::getParam(kCtrlName + "/thrust_weight_exp", dynamic_params_rot_.thrust_weight_exp);
  dh_ros::getParam(
    kCtrlName + "/thrust_rate_weight_exp", dynamic_params_rot_.thrust_rate_weight_exp);
}

void VelocityControllerRos::registerPublishers()
{
  rotor_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>("command/motor_speed", 1);
}

void VelocityControllerRos::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &VelocityControllerRos::eventCb, this);
  base_state_sub_ = nh_.subscribe("base_state", 1, &VelocityControllerRos::baseStateCb, this);
  battery_sub_ = nh_.subscribe("battery", 1, &VelocityControllerRos::batteryCb, this);
  if (is_transformable_)
  {
    joint_state_sub_ = nh_.subscribe("joint_states", 1, &VelocityControllerRos::jointStateCb, this);
  }
  cmd_sub_ = nh_.subscribe("command/velocity_yaw", 1, &VelocityControllerRos::commandCb, this);
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

  if (!cmd_received_)
  {
    return false;
  }

  return true;
}

void VelocityControllerRos::initialize()
{
  tar_rpy_.yaw = cur_bs_.pose.euler.yaw;  // ヨー角は初期状態を目標状態にする
  u_opt_ = VectorXd::Zero(z_rotors_.count());
}

void VelocityControllerRos::updateDynamicParams(const ConfigType& cfg)
{
  dynamic_params_vel_.natural_freq = cfg.natural_frequency;
  dynamic_params_vel_.damp_ratio = cfg.damping_ratio;

  dynamic_params_rot_.pred_horizon = cfg.prediction_horizon;
  dynamic_params_rot_.pred_steps = cfg.prediction_steps;
  dynamic_params_rot_.attitude_decay = cfg.attitude_decay;
  dynamic_params_rot_.heading_decay = cfg.heading_decay;
  dynamic_params_rot_.angvel_decay = cfg.angular_velocity_decay;
  dynamic_params_rot_.attitude_weight = cfg.attitude_weight;
  dynamic_params_rot_.heading_weight = cfg.heading_weight;
  dynamic_params_rot_.angvel_weight = cfg.angular_velocity_weight;
  dynamic_params_rot_.thrust_weight_exp = cfg.thrust_weight_exp;
  dynamic_params_rot_.thrust_rate_weight_exp = cfg.thrust_rate_weight_exp;
}

void VelocityControllerRos::runOnce()
{
  // 速度制御器
  const auto cur_vel_W = cur_bs_.pose.euler * cur_bs_.twist.vel;
  vel_controller_->update(cur_vel_W, tar_vel_W_, tar_acc_W_);

  // 非線形変換
  acc_controller_->update(tar_acc_W_, cur_bs_.pose.euler.yaw, U_, tar_rpy_.roll, tar_rpy_.pitch);
  const auto max_U = maxU();
  if (U_ < 0. || max_U < U_)
  {
    rosWarnThrottle(kWarnPeriod, "U_out = " << U_ << " is out of range [0, " << max_U << "].");
    U_ = dh_std::clamp(U_, 0., max_U);
  }

  // 姿勢制御器
  rot_controller_->update(
    cur_bs_.pose.euler, cur_bs_.twist.rot, q_, battery_.voltage, U_, tar_rpy_, u_opt_);

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
  assert(u.rows() == z_rotors_.count());

  for (uint32_t i = 0; i < u.rows(); ++i)
  {
    if (u(i) < -1.)
    {
      rosFatal("Negative thrust force: " << u(i) << " [N]");
      // TODO: 防御モードに移行
    }

    speeds.speeds[z_rotors_.rotorIdx(i)] = z_rotors_.thrustToRotSpeed(i, max(0., u(i)));
  }
}

double VelocityControllerRos::maxU()
{
  double max_U = 0.;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    max_U += z_rotors_.maxThrust(i, battery_.voltage);
  }
  return max_U;
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
      rosInfo("Velocity controller is ready.");
    }
    return;
  }

  // トピックが揃っていたら，状態を観測するたびに一回だけ制御器を回す．
  runOnce();
}

void VelocityControllerRos::batteryCb(const tobas_msgs::Battery& battery)
{
  if (!battery_received_)
  {
    battery_received_ = true;
  }

  battery_ = battery;
}

void VelocityControllerRos::jointStateCb(const sensor_msgs::JointState& js)
{
  if (js.name.size() != js.position.size())
  {
    rosError("The size of joint name and position is different.");
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
      rosError(e.what());
      return;
    }
  }

  if (!js_received_)
  {
    js_received_ = true;
  }
}

void VelocityControllerRos::commandCb(const CmdMsg& cmd)
{
  // コマンドレベルの処理
  if (cmd.level.data < cmd_level_)
  {
    rosErrorThrottle(
      kErrorPeriod, "The command is ignored because its level "
                      << cmd.level.data << "is lower than the current command level " << cmd_level_
                      << ".");
    return;
  }
  if (cmd.level.data > cmd_level_)
  {
    rosInfo("The command level is raised from " << cmd_level_ << " to " << cmd.level.data << ".");
    cmd_level_ = cmd.level.data;
  }

  // 目標速度と姿勢を更新
  switch (cmd.frame_id.data)
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
      rosError("Invalid FrameId: " << cmd.frame_id.data);
      return;
    }
  }

  if (!cmd_received_)
  {
    cmd_received_ = true;
  }
}

void VelocityControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!bs_received_)
  {
    rosWarn("Base state is not received yet.");
  }

  if (!battery_received_)
  {
    rosWarn("Battery state is not received yet.");
  }

  if (is_transformable_ && !js_received_)
  {
    rosWarn("Joint states are not received yet.");
  }

  if (!cmd_received_)
  {
    rosWarn("Command is not received yet.");
  }
}

void VelocityControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  updateDynamicParams(cfg);
  vel_controller_->reconfigure(dynamic_params_vel_);
  rot_controller_->reconfigure(dynamic_params_rot_);
  rosInfo("Dynamic parameters are updated.");
}
}  // namespace tobas_multirotor_controller
