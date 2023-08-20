#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/vector.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_eigen_tools/core.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_linear_control/util.hpp>

#include <tobas_tools/conversions/coordinates.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_fixed_wing_mpc/controller.hpp"
#include "../include/tobas_fixed_wing_mpc/constants.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;

namespace tobas_fixed_wing_mpc
{
Controller::Controller()
  : super(),
    x_rotors_(drone_, tobas::Axis::X_POSITIVE),
    eom_(drone_),
    check_topics_timer_(nh_, kCheckTopicsTimerPeriod, &Controller::checkTopicsTimerCb, this),
    server_(ros::NodeHandle(kCtrlName))
{
  getRosParams();
  drone_.loadFromParam(ns_);

  x_rotors_.updateInternalDataStructures();
  eom_.updateInternalDataStructures();

  if (x_rotors_.count() == 0)
  {
    rosthrow("The number of propellers is zero.");
  }

  q_0_.resize(drone_.tree().getNrOfJoints());
  c2d_.reset(new ctrl::C2D_RK4(eom_.kStateSize, eom_.inputSize()));

  pressure_received_ = false;
  battery_received_ = false;
  bs_received_ = false;
  state_ = State::START;
  rotor_speeds_msg_.speeds.resize(drone_.numRotors(), 0.);
  deflections_msg_.deflections.resize(drone_.numControlSurfaces(), 0.);
  feedback_msg_.trim_thrusts.resize(drone_.numRotors());
  feedback_msg_.delta_thrusts.resize(drone_.numRotors());
  feedback_msg_.trim_deflections.resize(drone_.numControlSurfaces());
  feedback_msg_.delta_deflections.resize(drone_.numControlSurfaces());

  mpc_.decay_time_consts.resize(kCtrlSize);
  setCz();
  setScales();
  mpc_.input_rate_weight.resize(eom_.inputSize());
  mpc_.input_weight.resize(eom_.inputSize());
  mpc_.control_weight.resize(kCtrlSize);
  setInputRateConstraint();
  mpc_.control_constraint.resize(kCtrlSize, 0);
  mpc_.current_state.resize(eom_.kStateSize);
  mpc_.set_state.resize(kCtrlSize);
  mpc_.last_input = VectorXd::Zero(eom_.inputSize());

  reconfigure(cfg_);

  registerPublishers();
  registerSubscribers();

  // Dynamic Reconfigure
  ConfigServer::CallbackType f = boost::bind(&Controller::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void Controller::getRosParams()
{
  dh_ros::getParam(kCtrlName + "/prediction_horizon", cfg_.prediction_horizon);
  dh_ros::getParam(kCtrlName + "/prediction_steps", cfg_.prediction_steps);

  dh_ros::getParam(kCtrlName + "/forward_speed_decay", cfg_.forward_speed_decay);
  dh_ros::getParam(kCtrlName + "/alpha_decay", cfg_.alpha_decay);
  dh_ros::getParam(kCtrlName + "/beta_decay", cfg_.beta_decay);
  dh_ros::getParam(kCtrlName + "/attitude_decay", cfg_.attitude_decay);
  dh_ros::getParam(kCtrlName + "/angular_velocity_decay", cfg_.angular_velocity_decay);

  dh_ros::getParam(kCtrlName + "/forward_speed_weight", cfg_.forward_speed_weight);
  dh_ros::getParam(kCtrlName + "/alpha_weight", cfg_.alpha_weight);
  dh_ros::getParam(kCtrlName + "/beta_weight", cfg_.beta_weight);
  dh_ros::getParam(kCtrlName + "/attitude_weight", cfg_.attitude_weight);
  dh_ros::getParam(kCtrlName + "/angular_velocity_weight", cfg_.angular_velocity_weight);

  dh_ros::getParam(kCtrlName + "/thrust_weight_exp", cfg_.thrust_weight_exp);
  dh_ros::getParam(kCtrlName + "/thrust_rate_weight_exp", cfg_.thrust_rate_weight_exp);
  dh_ros::getParam(kCtrlName + "/deflection_weight_exp", cfg_.deflection_weight_exp);
  dh_ros::getParam(kCtrlName + "/deflection_rate_weight_exp", cfg_.deflection_rate_weight_exp);
}

void Controller::registerPublishers()
{
  rotor_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>("command/motor_speed", 1);
  deflections_pub_ = nh_.advertise<tobas_msgs::ControlSurfaceDeflections>("command/deflections", 1);
  feedback_pub_ =
    nh_.advertise<tobas_msgs::FixedWingControllerFeedback>("fixed_wing_controller_feedback", 1);
}

void Controller::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &Controller::eventCb, this);
  air_pressure_sub_ = nh_.subscribe("air_pressure", 1, &Controller::airPressureCb, this);
  battery_sub_ = nh_.subscribe("battery", 1, &Controller::batteryCb, this);
  base_state_sub_ = nh_.subscribe("base_state", 1, &Controller::baseStateCb, this);
  cmd_sub_ = nh_.subscribe("command/speed_roll_delta_pitch", 1, &Controller::commandCb, this);
}

bool Controller::isReady()
{
  return pressure_received_ && battery_received_ && bs_received_;
}

void Controller::publishTakeoffCommand()
{
  // タイムスタンプを更新
  rotor_speeds_msg_.header.stamp = bs_ned_.header.stamp;
  deflections_msg_.header.stamp = bs_ned_.header.stamp;

  // 各ロータの回転数を発行
  for (uint32_t i = 0; i < x_rotors_.count(); ++i)
  {
    rotor_speeds_msg_.speeds[x_rotors_.rotorIdx(i)] =
      x_rotors_.rotSpeedFromVoltage(i, battery_.voltage);
  }
  rotor_speeds_pub_.publish(rotor_speeds_msg_);

  // 各操舵面の偏角を発行
  deflections_msg_.deflections[eom_.elevatorIndex()] = eom_.trimCondition().elevator();
  deflections_pub_.publish(deflections_msg_);
}

void Controller::setInitialTarget()
{
  const auto& trim = eom_.trimCondition();
  cmd_ned_.speed = trim.takeOffSpeed(air_density_);

  cmd_ned_.roll = 0.;
  cmd_ned_.delta_pitch = kInitialDeltaPitch;
}

void Controller::runOnce()
{
  // 状態方程式を更新
  eom_.update(cmd_ned_.speed, air_density_, battery_.voltage, q_0_);
  const ctrl::LinearDynamics cont(eom_.A(), eom_.B());
  const auto disc = c2d_->convert(cont, mpc_.time_step);
  fill(mpc_.discrete_dynamics, disc);

  setInputConstraint();  // EoMの更新後に呼ぶ必要がある
  updateCurrentStateVector();
  updateSetStateVector(cmd_ned_.roll, cmd_ned_.delta_pitch);

  // MPCを解いて最適制御入力を求める
  const auto du = mpc_.solveMPC();
  const auto u = eom_.trimInput() + du;

  // For debug
  // cout << "A_cont:" << endl << eom_.A() << endl;
  // cout << "B_cont:" << endl << eom_.B() << endl;
  // cout << "Discrete dynamics:" << endl << disc << endl;
  // cout << mpc_ << endl;

  // タイムスタンプを更新
  rotor_speeds_msg_.header.stamp = bs_ned_.header.stamp;
  deflections_msg_.header.stamp = bs_ned_.header.stamp;

  // 各ロータの回転数を発行
  const auto thrust = u.block(0, 0, x_rotors_.count(), 1);
  updateRotorSpeeds(thrust);
  rotor_speeds_pub_.publish(rotor_speeds_msg_);

  // 各操舵面の偏角を発行
  const auto deflections = u.block(x_rotors_.count(), 0, drone_.numControlSurfaces(), 1);
  updateDeflections(deflections);
  deflections_pub_.publish(deflections_msg_);

  // フィードバックを発行
  publishFeedback(du);
}

void Controller::setCz()
{
  mpc_.Cz = MatrixXd::Zero(kCtrlSize, eom_.kStateSize);

  mpc_.Cz(kCtrlIdx_u, eom_.kStateIdx_u) = 1;
  mpc_.Cz(kCtrlIdx_alpha, eom_.kStateIdx_alpha) = 1;
  mpc_.Cz(kCtrlIdx_beta, eom_.kStateIdx_beta) = 1;
  mpc_.Cz(kCtrlIdx_phi, eom_.kStateIdx_phi) = 1;
  mpc_.Cz(kCtrlIdx_theta, eom_.kStateIdx_theta) = 1;
  mpc_.Cz(kCtrlIdx_p, kCtrlIdx_p) = 1;
  mpc_.Cz(kCtrlIdx_q, kCtrlIdx_q) = 1;
  mpc_.Cz(kCtrlIdx_r, kCtrlIdx_r) = 1;
}

void Controller::setScales()
{
  // 状態変数のスケール
  mpc_.state_scale.resize(kCtrlSize);
  mpc_.state_scale(kCtrlIdx_u) = eom_.trimCondition().takeOffSpeed(kStandardAirDensity);
  mpc_.state_scale(kCtrlIdx_alpha) = drone_.vehicle().alpha_limit.range();
  mpc_.state_scale(kCtrlIdx_beta) = M_PI_4;
  mpc_.state_scale(kCtrlIdx_phi) = M_PI_4;
  mpc_.state_scale(kCtrlIdx_theta) = M_PI_4;
  mpc_.state_scale(kCtrlIdx_p) = M_PI;
  mpc_.state_scale(kCtrlIdx_q) = M_PI;
  mpc_.state_scale(kCtrlIdx_r) = M_PI;

  // 制御変数のスケール
  // 制御変数は状態変数と同じ
  mpc_.control_scale = mpc_.state_scale;

  // 制御入力のスケール
  mpc_.input_scale.resize(eom_.inputSize());
  const auto thrust_scale = tobas::getMass() * tobas::kGravity / x_rotors_.count();
  mpc_.input_scale.block(0, 0, x_rotors_.count(), 1).fill(thrust_scale);
  for (uint32_t i = 0; i < drone_.numControlSurfaces(); ++i)
  {
    mpc_.input_scale(x_rotors_.count() + i) = drone_.controlSurface(i).angle_limit.range();
  }
}

void Controller::setInputConstraint()
{
  const auto lb = eom_.minDeltaInput();
  const auto ub = eom_.maxDeltaInput();
  mpc_.input_constraint = ctrl::matIneqFromRange(lb, ub);
  // cout << mpc_.input_constraint << endl;
}

void Controller::setInputRateConstraint()
{
  VectorXd lb = VectorXd::Constant(eom_.inputSize(), numeric_limits<double>::lowest());
  VectorXd ub = VectorXd::Constant(eom_.inputSize(), numeric_limits<double>::max());

  // FIXME: 遅延が大きいなら舵角の変化率の制約は消してもいいかも
  for (uint32_t i = 0; i < drone_.numControlSurfaces(); ++i)
  {
    const auto& max_angle_rate = drone_.controlSurface(i).max_angle_rate;
    lb(x_rotors_.count() + i) = -max_angle_rate;
    ub(x_rotors_.count() + i) = +max_angle_rate;
  }

  mpc_.input_rate_constraint = ctrl::matIneqFromRange(lb, ub);
  // cout << mpc_.input_rate_constraint << endl;
}

void Controller::updateCurrentStateVector()
{
  const auto& trim = eom_.trimCondition();

  // TODO: 横系のトリムも考慮
  mpc_.current_state(eom_.kStateIdx_u) = bs_ned_.twist.vel.x() - trim.u();
  mpc_.current_state(eom_.kStateIdx_alpha) = tobas::angleOfAttack(bs_ned_.twist.vel) - trim.alpha();
  mpc_.current_state(eom_.kStateIdx_beta) = tobas::angleOfSideSlip(bs_ned_.twist.vel);
  mpc_.current_state(eom_.kStateIdx_phi) = bs_ned_.pose.euler.roll;
  mpc_.current_state(eom_.kStateIdx_theta) = bs_ned_.pose.euler.pitch - trim.theta();
  mpc_.current_state(eom_.kStateIdx_p) = bs_ned_.twist.rot.x();
  mpc_.current_state(eom_.kStateIdx_q) = bs_ned_.twist.rot.y();
  mpc_.current_state(eom_.kStateIdx_r) = bs_ned_.twist.rot.z();
}

void Controller::updateSetStateVector(double tar_roll, double tar_delta_pitch)
{
  const auto& trim = eom_.trimCondition();
  const auto tar_u = cmd_ned_.speed * cos(eom_.trimCondition().alpha());

  mpc_.set_state(kCtrlIdx_u) = tar_u - trim.u();
  mpc_.set_state(kCtrlIdx_alpha) = 0.;
  mpc_.set_state(kCtrlIdx_beta) = 0.;
  mpc_.set_state(kCtrlIdx_phi) = tar_roll;
  mpc_.set_state(kCtrlIdx_theta) = tar_delta_pitch;
  mpc_.set_state(kCtrlIdx_p) = 0.;
  mpc_.set_state(kCtrlIdx_q) = 0.;
  mpc_.set_state(kCtrlIdx_r) = 0.;
}

void Controller::updateRotorSpeeds(const VectorXd& thrust)
{
  ROS_ASSERT(thrust.rows() == x_rotors_.count());

  for (uint32_t i = 0; i < thrust.rows(); ++i)
  {
    if (thrust(i) < -1.)
    {
      rosFatal("Negative thrust force: " << thrust(i) << " [N]");
      // TODO: 防御モードに移行
    }

    rotor_speeds_msg_.speeds[x_rotors_.rotorIdx(i)] =
      x_rotors_.rotSpeedFromThrust(i, max(0., thrust(i)));
  }
}

void Controller::updateDeflections(const VectorXd& deflections)
{
  ROS_ASSERT(deflections.rows() == drone_.numControlSurfaces());

  deflections_msg_.deflections = eigen_tools::toStdVector(deflections);
}

void Controller::publishFeedback(const Eigen::VectorXd& du)
{
  const auto& trim = eom_.trimCondition();

  feedback_msg_.trim_u = trim.u();
  feedback_msg_.trim_alpha = trim.alpha();

  for (uint32_t i = 0; i < x_rotors_.count(); ++i)
  {
    feedback_msg_.trim_thrusts[x_rotors_.rotorIdx(i)] = eom_.trimInput()(i);
    feedback_msg_.delta_thrusts[x_rotors_.rotorIdx(i)] = du(i);
  }

  for (uint32_t i = 0; i < drone_.numControlSurfaces(); ++i)
  {
    const auto u_idx = x_rotors_.count() + i;
    feedback_msg_.trim_deflections[i] = eom_.trimInput()[u_idx];
    feedback_msg_.delta_deflections[i] = du(u_idx);
  }

  feedback_pub_.publish(feedback_msg_);
}

void Controller::reconfigure(const ConfigType& cfg)
{
  ROS_ASSERT(cfg.prediction_horizon > 0.);
  ROS_ASSERT(cfg.prediction_steps > 0);
  ROS_ASSERT(cfg.forward_speed_decay >= 0.);
  ROS_ASSERT(cfg.alpha_decay >= 0.);
  ROS_ASSERT(cfg.beta_decay >= 0.);
  ROS_ASSERT(cfg.attitude_decay >= 0.);
  ROS_ASSERT(cfg.angular_velocity_decay >= 0.);
  ROS_ASSERT(cfg.forward_speed_weight > 0.);
  ROS_ASSERT(cfg.alpha_weight > 0.);
  ROS_ASSERT(cfg.beta_weight > 0.);
  ROS_ASSERT(cfg.attitude_weight > 0.);
  ROS_ASSERT(cfg.angular_velocity_weight > 0.);

  mpc_.time_step = cfg.prediction_horizon / cfg.prediction_steps;
  mpc_.prediction_steps = mpc_.input_steps = cfg.prediction_steps;
  mpc_.decay_time_consts(kCtrlIdx_u) = cfg.forward_speed_decay;
  mpc_.decay_time_consts(kCtrlIdx_alpha) = cfg.alpha_decay;
  mpc_.decay_time_consts(kCtrlIdx_beta) = cfg.beta_decay;
  mpc_.decay_time_consts(kCtrlIdx_phi) = cfg.attitude_decay;
  mpc_.decay_time_consts(kCtrlIdx_theta) = cfg.attitude_decay;
  mpc_.decay_time_consts(kCtrlIdx_p) = cfg.angular_velocity_decay;
  mpc_.decay_time_consts(kCtrlIdx_q) = cfg.angular_velocity_decay;
  mpc_.decay_time_consts(kCtrlIdx_r) = cfg.angular_velocity_decay;

  const ctrl::LinearDynamics cont(eom_.A(), eom_.B());
  const auto disc = c2d_->convert(cont, mpc_.time_step);
  mpc_.discrete_dynamics.resize(cfg.prediction_steps, disc);

  // 制御変数の重み
  mpc_.control_weight(kCtrlIdx_u) = cfg.forward_speed_weight;
  mpc_.control_weight(kCtrlIdx_alpha) = cfg.alpha_weight;
  mpc_.control_weight(kCtrlIdx_beta) = cfg.beta_weight;
  mpc_.control_weight(kCtrlIdx_phi) = cfg.attitude_weight;
  mpc_.control_weight(kCtrlIdx_theta) = cfg.attitude_weight;
  mpc_.control_weight(kCtrlIdx_p) = cfg.angular_velocity_weight;
  mpc_.control_weight(kCtrlIdx_q) = cfg.angular_velocity_weight;
  mpc_.control_weight(kCtrlIdx_r) = cfg.angular_velocity_weight;

  // 制御入力の重み
  mpc_.input_weight.topRows(x_rotors_.count()).fill(pow(10, cfg.thrust_weight_exp));
  mpc_.input_weight.bottomRows(drone_.numControlSurfaces())
    .fill(pow(10, cfg.deflection_weight_exp));

  // 制御入力の変化率の重み
  mpc_.input_rate_weight.topRows(x_rotors_.count()).fill(pow(10, cfg.thrust_rate_weight_exp));
  mpc_.input_rate_weight.bottomRows(drone_.numControlSurfaces())
    .fill(pow(10, cfg.deflection_rate_weight_exp));
}

void Controller::eventCb(const tobas_msgs::Event& event)
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

void Controller::airPressureCb(const sensor_msgs::FluidPressure& msg)
{
  if (!pressure_received_)
  {
    pressure_received_ = true;
  }

  air_density_ = dh_std::pressureToDensity(msg.fluid_pressure);
}

void Controller::batteryCb(const tobas_msgs::Battery& battery)
{
  if (!battery_received_)
  {
    battery_received_ = true;
  }

  battery_ = battery;
}

void Controller::baseStateCb(const StateMsg& bs_nwu)
{
  if (!bs_received_)
  {
    bs_received_ = true;
  }

  // コールバックの時点で全てNED座標系に変換しておく
  tf::baseStateNwuToNed(bs_nwu, bs_ned_);

  switch (state_)
  {
    case START:
    {
      if (isReady())
      {
        rosInfo("Controller is ready.");
        check_topics_timer_.stop();
        state_ = TAKEOFF;
      }
      break;
    }
    case TAKEOFF:
    {
      const auto cur_V = bs_ned_.twist.vel.Norm();
      const auto min_V = eom_.trimCondition().minimumSpeed(air_density_);
      const auto eom_error = eom_.update(max(cur_V, min_V), air_density_, battery_.voltage, q_0_);
      if (eom_error < 0)
      {
        rosError(eom_.errorMessage());
      }

      publishTakeoffCommand();

      // 最低速度を上回ったら制御開始
      const auto cur_speed = bs_nwu.twist.vel.Norm();
      if (cur_speed > eom_.trimCondition().minimumSpeed(air_density_))
      {
        setInitialTarget();
        state_ = FLIGHT;
        rosInfo("The aircraft takes off and begins flight control.");
      }
      break;
    }
    case FLIGHT:
    {
      runOnce();
      break;
    }
    case LANDING:
    {
      // TODO
      break;
    }
  }
}

void Controller::commandCb(const CmdMsg& cmd_nwu)
{
  if (!(state_ == FLIGHT))
  {
    rosError("Not in flight state.");
    return;
  }

  if (!eom_.trimCondition().speedLimit(air_density_).inRange(cmd_nwu.speed))
  {
    rosError("Invalid speed is commanded.");
    return;
  }

  tf::speedRollDeltaPitchNwuToNed(cmd_nwu, cmd_ned_);
}

void Controller::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!pressure_received_)
  {
    rosWarn("Air pressure is not received yet.");
  }

  if (!battery_received_)
  {
    rosWarn("Battery state is not received yet.");
  }

  if (!bs_received_)
  {
    rosWarn("Base state is not received yet.");
  }
}

void Controller::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  reconfigure(cfg);
}
}  // namespace tobas_fixed_wing_mpc
