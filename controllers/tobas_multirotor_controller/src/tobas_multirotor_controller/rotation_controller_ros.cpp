#include <dh_std_tools/vector.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include "../../include/tobas_multirotor_controller/rotation_controller_ros.hpp"
#include "../../include/tobas_multirotor_controller/constants.hpp"

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace tobas_multirotor_controller
{
RotationControllerRos::RotationControllerRos()
  : super(),
    jnt_name_parser_(drone_.tree()),
    z_rotors_(drone_, tobas::Axis::Z_POSITIVE),
    cmd_level_(tobas_msgs::CommandLevel::NORMAL),
    is_initialized_(false),
    battery_received_(false),
    bs_received_(false),
    js_received_(false),
    rpy_thrust_received_(false),
    check_topics_timer_(
      nh_,
      kCheckTopicsTimerPeriod,
      &RotationControllerRos::checkTopicsTimerCb,
      this),
    server_(ros::NodeHandle(kCtrlName))
{
  getRosParams();
  drone_.loadFromParam(ns_);

  jnt_name_parser_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();

  rot_controller_.reset(new RotationController(drone_, dynamic_params_rot_));

  is_transformable_ = drone_.postureDefiningJoints().size() > 0;
  q_.resize(drone_.tree().getNrOfJoints());
  rotor_speeds_.speeds.resize(drone_.numRotors(), 0.);

  registerPublishers();
  registerSubscribers();

  // Dynamic Reconfigure
  ConfigServer::CallbackType f =
    boost::bind(&RotationControllerRos::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void RotationControllerRos::getRosParams()
{
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

void RotationControllerRos::registerPublishers()
{
  rotor_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>("command/motor_speed", 1);
}

void RotationControllerRos::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &RotationControllerRos::eventCb, this);
  battery_sub_ = nh_.subscribe("battery", 1, &RotationControllerRos::batteryCb, this);
  base_state_sub_ = nh_.subscribe("base_state", 1, &RotationControllerRos::baseStateCb, this);
  if (is_transformable_)
  {
    joint_state_sub_ = nh_.subscribe("joint_states", 1, &RotationControllerRos::jointStateCb, this);
  }
  rpy_thrust_sub_ =
    nh_.subscribe("command/rpy_thrust", 1, &RotationControllerRos::rpyThrustCb, this);
}

bool RotationControllerRos::isReady()
{
  if (!battery_received_)
    return false;

  if (!bs_received_)
    return false;

  if (is_transformable_ && !js_received_)
    return false;

  if (!rpy_thrust_received_)
    return false;

  return true;
}

void RotationControllerRos::initialize()
{
  u_opt_ = VectorXd::Zero(z_rotors_.count());
}

void RotationControllerRos::updateDynamicParams(const ConfigType& cfg)
{
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

void RotationControllerRos::runOnce()
{
  // 姿勢制御器
  rot_controller_->update(
    bs_.pose.euler, bs_.twist.rot, q_, battery_.voltage, rpy_thrust_.thrust, rpy_thrust_.rpy,
    u_opt_);

  // 各モータの回転速度メッセージを更新
  rotor_speeds_.header.stamp = bs_.header.stamp;
  ctrlInputToRotorSpeeds(u_opt_, rotor_speeds_);

  // モータ速度を発行
  rotor_speeds_pub_.publish(rotor_speeds_);
}

void RotationControllerRos::ctrlInputToRotorSpeeds(
  const VectorXd& u,
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

double RotationControllerRos::maxU()
{
  double max_U = 0.;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    max_U += z_rotors_.maxThrust(i, battery_.voltage);
  }
  return max_U;
}

void RotationControllerRos::eventCb(const tobas_msgs::Event& event)
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

void RotationControllerRos::batteryCb(const tobas_msgs::Battery& battery)
{
  if (!battery_received_)
  {
    battery_received_ = true;
  }

  battery_ = battery;
}

void RotationControllerRos::baseStateCb(const tobas_msgs::BaseState& bs)
{
  if (!bs_received_)
  {
    bs_received_ = true;
  }

  bs_ = bs;

  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      initialize();
      is_initialized_ = true;
      rosInfo("Rotation controller is ready.");
    }
    return;
  }

  // トピックが揃っていたら，状態を観測するたびに一回だけ制御器を回す．
  runOnce();
}

void RotationControllerRos::jointStateCb(const sensor_msgs::JointState& js)
{
  if (js.name.size() != js.position.size())
  {
    rosError("The size of joint name and position is different.");
    return;
  }

  for (const auto& jnt_name : drone_.postureDefiningJoints())
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

void RotationControllerRos::rpyThrustCb(const tobas_msgs::RollPitchYawThrust& rpy_thrust)
{
  // コマンドレベルの処理
  if (rpy_thrust.level.data < cmd_level_)
  {
    rosErrorThrottle(
      kErrorPeriod, "The command is ignored because its level "
                      << rpy_thrust.level.data << "is lower than the current command level "
                      << cmd_level_ << ".");
    return;
  }
  if (rpy_thrust.level.data > cmd_level_)
  {
    rosInfo(
      "The command level is raised from " << cmd_level_ << " to " << rpy_thrust.level.data << ".");
    cmd_level_ = rpy_thrust.level.data;
  }

  // 目標姿勢 & 目標推力の範囲チェックをしながら更新
  // Roll
  if (abs(rpy_thrust.rpy.roll) > kMaxAttitude)
  {
    rosErrorThrottle(
      kErrorPeriod, "roll = " << rpy_thrust.rpy.roll << " is out of range [" << -kMaxAttitude
                              << ", " << kMaxAttitude << "].");
  }
  rpy_thrust_.rpy.roll = clamp(rpy_thrust.rpy.roll, -kMaxAttitude, kMaxAttitude);

  // Pitch
  if (abs(rpy_thrust.rpy.pitch) > kMaxAttitude)
  {
    rosErrorThrottle(
      kErrorPeriod, "pitch = " << rpy_thrust.rpy.pitch << " is out of range [" << -kMaxAttitude
                               << ", " << kMaxAttitude << "].");
  }
  rpy_thrust_.rpy.pitch = clamp(rpy_thrust.rpy.pitch, -kMaxAttitude, kMaxAttitude);

  // Yaw
  rpy_thrust_.rpy.yaw = rpy_thrust.rpy.yaw;

  // Thrust
  const auto max_U = maxU();
  if (rpy_thrust.thrust < 0. || max_U < rpy_thrust.thrust)
  {
    rosWarnThrottle(
      kWarnPeriod, "thrust = " << rpy_thrust.thrust << " is out of range [0, " << max_U << "].");
  }
  rpy_thrust_.thrust = clamp(rpy_thrust.thrust, 0., max_U);

  if (!rpy_thrust_received_)
  {
    rpy_thrust_received_ = true;
  }
}

void RotationControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!battery_received_)
    rosWarn("Battery state is not received yet.");

  if (!bs_received_)
    rosWarn("Base state is not received yet.");

  if (is_transformable_ && !js_received_)
    rosWarn("Joint states are not received yet.");

  if (!rpy_thrust_received_)
    rosWarn("RPY & Thrust command is not received yet.");
}

void RotationControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  updateDynamicParams(cfg);
  rot_controller_->reconfigure(dynamic_params_rot_);
  rosInfo("Dynamic parameters are updated.");
}
}  // namespace tobas_multirotor_controller
