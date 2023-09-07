#include <dh_std_tools/vector.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_multirotor_controller/rotation_controller_ros.hpp"
#include "../include/tobas_multirotor_controller/constants.hpp"

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace tobas_multirotor_controller
{
RotationControllerRos::RotationControllerRos(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name),
    jnt_name_parser_(drone_.tree()),
    z_rotors_(drone_, tobas::Axis::Z_POSITIVE),
    rot_controller_(drone_),
    is_initialized_(false),
    battery_received_(false),
    pt_received_(false),
    js_received_(false),
    rpy_thrust_received_(false),
    rpyd_thrust_received_(false),
    check_topics_timer_(
      nh_,
      kCheckTopicsTimerPeriod,
      &RotationControllerRos::checkTopicsTimerCb,
      this),
    server_(ros::NodeHandle(kCtrlName))
{
  // Dynamic Reconfigure
  // ConfigServer::CallbackType f =
  //   boost::bind(&RotationControllerRos::dynamicReconfigureCb, this, _1, _2);
  // server_.setCallback(f);

  getRosParams();
  drone_.loadFromParam(nh_);

  jnt_name_parser_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();

  rot_controller_.updateInternalDataStructures();
  rot_controller_.configure(dynamic_params_);

  is_transformable_ = drone_.postureDefiningJoints().size() > 0;
  q_.resize(drone_.tree().getNrOfJoints());
  rpy_thrust_.level.data = tobas_msgs::CommandLevel::NORMAL;

  registerPublishers();
  registerSubscribers();
}

void RotationControllerRos::getRosParams()
{
  dh_ros::getParam(nh_, kCtrlName + "/prediction_horizon", dynamic_params_.pred_horizon);
  dh_ros::getParam(nh_, kCtrlName + "/prediction_steps", dynamic_params_.pred_steps);
  dh_ros::getParam(nh_, kCtrlName + "/attitude_decay", dynamic_params_.attitude_decay);
  dh_ros::getParam(nh_, kCtrlName + "/heading_decay", dynamic_params_.heading_decay);
  dh_ros::getParam(nh_, kCtrlName + "/angular_velocity_decay", dynamic_params_.angvel_decay);
  dh_ros::getParam(nh_, kCtrlName + "/attitude_weight", dynamic_params_.attitude_weight);
  dh_ros::getParam(nh_, kCtrlName + "/heading_weight", dynamic_params_.heading_weight);
  dh_ros::getParam(nh_, kCtrlName + "/angular_velocity_weight", dynamic_params_.angvel_weight);
  dh_ros::getParam(nh_, kCtrlName + "/thrust_weight_exp", dynamic_params_.thrust_weight_exp);
  dh_ros::getParam(
    nh_, kCtrlName + "/thrust_rate_weight_exp", dynamic_params_.thrust_rate_weight_exp);
}

void RotationControllerRos::registerPublishers()
{
  rotor_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>("command/motor_speed", 1);
}

void RotationControllerRos::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &RotationControllerRos::eventCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe("battery", 1, &RotationControllerRos::batteryCb, this, tcpNoDelay());
  pt_sub_ = nh_.subscribe("pose_twist", 1, &RotationControllerRos::poseTwistCb, this, tcpNoDelay());
  if (is_transformable_)
  {
    joint_state_sub_ =
      nh_.subscribe("joint_states", 1, &RotationControllerRos::jointStateCb, this, tcpNoDelay());
  }
  rpy_thrust_sub_ =
    nh_.subscribe("command/rpy_thrust", 1, &RotationControllerRos::rpyThrustCb, this, tcpNoDelay());
  rpyd_thrust_sub_ = nh_.subscribe(
    "command/roll_pitch_yawrate_thrust", 1, &RotationControllerRos::rpydThrustCb, this,
    tcpNoDelay());
}

bool RotationControllerRos::isReady()
{
  if (!battery_received_)
    return false;

  if (!pt_received_)
    return false;

  if (is_transformable_ && !js_received_)
    return false;

  if (!rpy_thrust_received_ && !rpyd_thrust_received_)
    return false;

  return true;
}

void RotationControllerRos::initialize()
{
  u_opt_ = VectorXd::Zero(z_rotors_.count());
}

void RotationControllerRos::updateDynamicParams(const ConfigType& cfg)
{
  dynamic_params_.pred_horizon = cfg.prediction_horizon;
  dynamic_params_.pred_steps = cfg.prediction_steps;
  dynamic_params_.attitude_decay = cfg.attitude_decay;
  dynamic_params_.heading_decay = cfg.heading_decay;
  dynamic_params_.angvel_decay = cfg.angular_velocity_decay;
  dynamic_params_.attitude_weight = cfg.attitude_weight;
  dynamic_params_.heading_weight = cfg.heading_weight;
  dynamic_params_.angvel_weight = cfg.angular_velocity_weight;
  dynamic_params_.thrust_weight_exp = cfg.thrust_weight_exp;
  dynamic_params_.thrust_rate_weight_exp = cfg.thrust_rate_weight_exp;
}

void RotationControllerRos::runOnce()
{
  // 姿勢制御器
  try
  {
    // stopwatch_.start();
    rot_controller_.update(
      pt_->pose.euler, pt_->twist.rot, q_, battery_->voltage, rpy_thrust_.thrust, rpy_thrust_.rpy,
      u_opt_);
    // stopwatch_.stop();
  }
  catch (const exception& e)  // MPCがコケたり
  {
    rosError(name_, e.what());
    return;
  }

  // モータ速度メッセージを作成
  const auto rotor_speeds = boost::make_shared<tobas_msgs::RotorSpeeds>();
  rotor_speeds->header.stamp = pt_->header.stamp;
  rotor_speeds->speeds.resize(drone_.numRotors(), 0.);
  for (uint32_t i = 0; i < u_opt_.rows(); ++i)
  {
    if (u_opt_(i) < -1.)
    {
      rosFatal(name_, "Negative thrust force: " << u_opt_(i) << " [N]");
      // TODO: 防御モードに移行
    }
    rotor_speeds->speeds[z_rotors_.rotorIdx(i)] =
      z_rotors_.rotSpeedFromThrust(i, max(0., u_opt_(i)));
  }

  // モータ速度を発行
  rotor_speeds_pub_.publish(rotor_speeds);
}

double RotationControllerRos::maxThrustSum()
{
  double res = 0.;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    res += z_rotors_.thrustFromVoltage(i, battery_->voltage);
  }
  return res;
}

double RotationControllerRos::minThrustSum()
{
  const auto min_voltage = battery_->voltage * tobas::kMotorSpinArm;
  double res = 0.;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    res += z_rotors_.thrustFromVoltage(i, min_voltage);
  }
  return res;
}

bool RotationControllerRos::isCommandLevelOk(const tobas_msgs::CommandLevel& level)
{
  if (level.data < rpy_thrust_.level.data)
  {
    rosErrorThrottle(
      kErrorPeriod, name_,
      "The command is ignored because its level "
        << static_cast<int>(level.data) << "is lower than the current command level "
        << static_cast<int>(rpy_thrust_.level.data) << ".");
    return false;
  }

  if (level.data > rpy_thrust_.level.data)
  {
    rosInfo(
      name_, "The command level is raised from " << static_cast<int>(rpy_thrust_.level.data)
                                                 << " to " << static_cast<int>(level.data) << ".");
    rpy_thrust_.level = level;
  }

  return true;
}

void RotationControllerRos::updateTargetRoll(double tar_roll)
{
  if (abs(tar_roll) > kMaxAttitude)
  {
    rosErrorThrottle(
      kErrorPeriod, name_,
      "roll = " << tar_roll << " is out of range [" << -kMaxAttitude << ", " << kMaxAttitude
                << "].");
  }
  rpy_thrust_.rpy.roll = clamp(tar_roll, -kMaxAttitude, kMaxAttitude);
}

void RotationControllerRos::updateTargetPitch(double tar_pitch)
{
  if (abs(tar_pitch) > kMaxAttitude)
  {
    rosErrorThrottle(
      kErrorPeriod, name_,
      "pitch = " << tar_pitch << " is out of range [" << -kMaxAttitude << ", " << kMaxAttitude
                 << "].");
  }
  rpy_thrust_.rpy.pitch = clamp(tar_pitch, -kMaxAttitude, kMaxAttitude);
}

void RotationControllerRos::updateTargetThrust(double tar_thrust)
{
  const auto max_U = maxThrustSum();
  const auto min_U = minThrustSum();
  if (tar_thrust < min_U || max_U < tar_thrust)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "thrust = " << tar_thrust << " is out of range [" << min_U << ", " << max_U << "].");
  }
  rpy_thrust_.thrust = clamp(tar_thrust, min_U, max_U);
}

void RotationControllerRos::eventCb(const tobas_msgs::EventConstPtr& event)
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

void RotationControllerRos::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  if (!battery_received_)
  {
    battery_received_ = true;
  }

  battery_ = battery;
}

void RotationControllerRos::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  if (!pt_received_)
  {
    pt_received_ = true;
  }

  pt_ = pt;

  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      initialize();
      is_initialized_ = true;
      rosInfo(name_, "Rotation controller is ready.");
    }
    return;
  }

  // トピックが揃っていたら，状態を観測するたびに一回だけ制御器を回す．
  runOnce();
}

void RotationControllerRos::jointStateCb(const sensor_msgs::JointStateConstPtr& js)
{
  if (js->name.size() != js->position.size())
  {
    rosError(name_, "The size of joint name and position is different.");
    return;
  }

  for (const auto& jnt_name : drone_.postureDefiningJoints())
  {
    try
    {
      const auto msg_idx = dh_std::findIndex(js->name, jnt_name);  // msg内でのインデックス
      const auto& jnt_pos = js->position[msg_idx];
      const auto& kdl_idx = jnt_name_parser_.jointIndex(jnt_name);  // Tree内でのインデックス
      q_(kdl_idx) = jnt_pos;
    }
    catch (const exception& e)
    {
      rosError(name_, e.what());
      return;
    }
  }

  if (!js_received_)
  {
    js_received_ = true;
  }
}

void RotationControllerRos::rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpy_thrust)
{
  if (!pt_received_)
  {
    return;
  }

  if (!isCommandLevelOk(rpy_thrust->level))
  {
    return;
  }

  updateTargetRoll(rpy_thrust->rpy.roll);
  updateTargetPitch(rpy_thrust->rpy.pitch);
  updateTargetThrust(rpy_thrust->thrust);

  // Yawの目標値を更新
  rpy_thrust_.rpy.yaw = rpy_thrust->rpy.yaw;

  if (!rpy_thrust_received_)
  {
    rpy_thrust_received_ = true;
  }
}

void RotationControllerRos::rpydThrustCb(
  const tobas_msgs::RollPitchYawrateThrustConstPtr& rpyd_thrust)
{
  if (!pt_received_)
  {
    return;
  }

  if (!isCommandLevelOk(rpyd_thrust->level))
  {
    return;
  }

  updateTargetRoll(rpyd_thrust->roll);
  updateTargetPitch(rpyd_thrust->pitch);
  updateTargetThrust(rpyd_thrust->thrust);

  // Yawの目標値を更新
  if (rpyd_thrust_received_)
  {
    const ros::Time now = ros::Time::now();
    const auto dt = (now - t_last_rpyd_thrust_).toSec();
    t_last_rpyd_thrust_ = now;
    if (dt > kRollPitchYawrateThrustTimeout)
    {
      rosInfo(
        name_, "The time gap from the previous command is over "
                 << kRollPitchYawrateThrustTimeout << " seconds. The command is reset.");
      rpyd_thrust_received_ = false;
      return;
    }
    rpy_thrust_.rpy.yaw += rpyd_thrust->yawrate * dt;
  }
  else
  {
    t_last_rpyd_thrust_ = ros::Time::now();
    rpy_thrust_.rpy.yaw = pt_->pose.euler.yaw;  // 最初は現在のヨー角を指令
    rpyd_thrust_received_ = true;
  }
}

void RotationControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!battery_received_)
    rosWarn(name_, "Battery state is not received yet.");

  if (!pt_received_)
    rosWarn(name_, "Pose & Twist is not received yet.");

  if (is_transformable_ && !js_received_)
    rosWarn(name_, "Joint states are not received yet.");

  // コマンドの警告はしない
}

void RotationControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  updateDynamicParams(cfg);
  rot_controller_.configure(dynamic_params_);
  rosInfo(name_, "Dynamic parameters are updated.");
}
}  // namespace tobas_multirotor_controller
