#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/vector.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_eigen_tools/core.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_linear_control/util.hpp>

#include <tobas_tools/conversions/coordinates.hpp>

#include "../../include/tobas_fixed_wing_controller/controller.hpp"
#include "../../include/tobas_fixed_wing_controller/constants.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;

namespace tobas_fixed_wing_controller
{
Controller::Controller()
  : super(),
    x_rotors_(drone_, tobas::Axis::X_POSITIVE),
    eom_(drone_),
    server_(ros::NodeHandle(kCtrlName))
{
  getRosParams();
  drone_.loadFromParam(ns_);

  x_rotors_.updateInternalDataStructures();
  eom_.updateInternalDataStructures();

  q_0_.resize(drone_.tree().getNrOfJoints());

  c2d_.reset(new ctrl::C2D_RK4(eom_.kStateSize, eom_.inputSize()));

  pressure_received_ = false;
  bs_received_ = false;
  state_ = State::START;
  rotor_speeds_msg_.speeds.resize(drone_.numRotors(), 0.);
  deflections_msg_.deflections.resize(drone_.numControlSurfaces(), 0.);

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
  createTimers();

  // Dynamic Reconfigure
  ConfigServer::CallbackType f = boost::bind(&Controller::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void Controller::getRosParams()
{
  dh_ros::getParam(kCtrlName + "/prediction_horizon", cfg_.prediction_horizon);
  dh_ros::getParam(kCtrlName + "/prediction_steps", cfg_.prediction_steps);
  dh_ros::getParam(kCtrlName + "/beta_decay", cfg_.beta_decay);
  dh_ros::getParam(kCtrlName + "/attitude_decay", cfg_.attitude_decay);
  dh_ros::getParam(kCtrlName + "/beta_weight", cfg_.beta_weight);
  dh_ros::getParam(kCtrlName + "/attitude_weight", cfg_.attitude_weight);
  dh_ros::getParam(kCtrlName + "/thrust_weight_exp", cfg_.thrust_weight_exp);
  dh_ros::getParam(kCtrlName + "/thrust_rate_weight_exp", cfg_.thrust_rate_weight_exp);
  dh_ros::getParam(kCtrlName + "/deflection_weight_exp", cfg_.deflection_weight_exp);
  dh_ros::getParam(kCtrlName + "/deflection_rate_weight_exp", cfg_.deflection_rate_weight_exp);
}

void Controller::registerPublishers()
{
  rotor_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>("command/motor_speed", 1, false);
  deflections_pub_ =
    nh_.advertise<tobas_msgs::ControlSurfaceDeflections>("command/deflections", 1, false);
}

void Controller::registerSubscribers()
{
  air_pressure_sub_ = nh_.subscribe("air_pressure", 1, &Controller::airPressureCb, this);
  base_state_sub_ = nh_.subscribe("base_state", 1, &Controller::baseStateCb, this);
  cmd_sub_ = nh_.subscribe("command/speed_roll_delta_pitch", 1, &Controller::commandCb, this);
}

void Controller::createTimers()
{
  check_topics_timer_ =
    nh_.createTimer(ros::Duration(kCheckTopicsTimerPeriod), &Controller::checkTopicsTimerCb, this);
}

bool Controller::isReady()
{
  return pressure_received_ && bs_received_;
}

void Controller::publishTakeoffCommand()
{
  // タイムスタンプを更新
  rotor_speeds_msg_.header.stamp = bs_ned_.header.stamp;
  deflections_msg_.header.stamp = bs_ned_.header.stamp;

  // 各ロータの回転数を発行
  for (int i = 0; i < x_rotors_.count(); ++i)
  {
    rotor_speeds_msg_.speeds[x_rotors_.rotorIdx(i)] = x_rotors_.maxRotSpeed(i);
  }
  rotor_speeds_pub_.publish(rotor_speeds_msg_);

  // 各操舵面の偏角を発行
  deflections_msg_.deflections[eom_.elevatorIndex()] = eom_.trimCondition().elevator();
  deflections_pub_.publish(deflections_msg_);
}

void Controller::setInitialTarget()
{
  const auto& trim = eom_.trimCondition();
  cmd_ned_.speed = trim.speedLimit(air_density_).lower * 1.1;  // TODO: 初期速度をどう決めるか

  cmd_ned_.roll = 0.;
  cmd_ned_.delta_pitch = kInitialDeltaPitch;
}

void Controller::runOnce()
{
  // 状態方程式を更新
  eom_.update(cmd_ned_.speed, air_density_, q_0_);
  const ctrl::LinearDynamics cont(eom_.A(), eom_.B());
  const auto disc = c2d_->convert(cont, mpc_.time_step);
  fill(mpc_.discrete_dynamics, disc);

  setInputConstraint();  // EoMの更新後に呼ぶ必要がある
  updateCurrentStateVector();
  updateSetStateVector(cmd_ned_.roll, cmd_ned_.delta_pitch);

  // MPCを解いて最適制御入力を求める
  const auto du = mpc_.solveMPC();
  const auto u = eom_.trimInput() + du;

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
}

void Controller::setCz()
{
  mpc_.Cz = MatrixXd::Zero(kCtrlSize, eom_.kStateSize);

  mpc_.Cz(kCtrlIdx_u, eom_.kStateIdx_u) = 1;
  mpc_.Cz(kCtrlIdx_alpha, eom_.kStateIdx_alpha) = 1;
  mpc_.Cz(kCtrlIdx_beta, eom_.kStateIdx_beta) = 1;
  mpc_.Cz(kCtrlIdx_phi, eom_.kStateIdx_phi) = 1;
  mpc_.Cz(kCtrlIdx_theta, eom_.kStateIdx_theta) = 1;
}

void Controller::setScales()
{
  // 制御変数のスケール
  mpc_.control_scale.resize(kCtrlSize);
  mpc_.control_scale(kCtrlIdx_u) = eom_.trimCondition().speedLimit(kStandardAirDensity).lower;
  mpc_.control_scale(kCtrlIdx_alpha) = mpc_.control_scale(kCtrlIdx_beta) = M_PI;
  mpc_.control_scale(kCtrlIdx_phi) = mpc_.control_scale(kCtrlIdx_theta) = M_PI;

  // 制御入力のスケール
  mpc_.input_scale.resize(eom_.inputSize());
  for (int i = 0; i < x_rotors_.count(); ++i)
  {
    mpc_.input_scale(i) = x_rotors_.maxThrust(i);
  }
  mpc_.input_scale.block(x_rotors_.count(), 0, drone_.numControlSurfaces(), 1) =
    VectorXd::Constant(drone_.numControlSurfaces(), M_PI);
}

void Controller::setInputConstraint()
{
  const auto lb = eom_.minDeltaInput();
  const auto ub = eom_.maxDeltaInput();
  mpc_.input_constraint = ctrl::matIneqFromRange(lb, ub);
}

void Controller::setInputRateConstraint()
{
  VectorXd lb = VectorXd::Constant(eom_.inputSize(), numeric_limits<double>::lowest());
  VectorXd ub = VectorXd::Constant(eom_.inputSize(), numeric_limits<double>::max());

  // FIXME: 遅延が大きいなら舵角の変化率の制約は消してもいいかも
  for (int i = 0; i < drone_.numControlSurfaces(); ++i)
  {
    const auto& max_angle_rate = drone_.controlSurface(i).max_angle_rate;
    lb(x_rotors_.count() + i) = -max_angle_rate;
    ub(x_rotors_.count() + i) = +max_angle_rate;
  }

  mpc_.input_rate_constraint = ctrl::matIneqFromRange(lb, ub);
}

void Controller::updateCurrentStateVector()
{
  const KDL::Vector linvel_B = bs_ned_.pose.euler * bs_ned_.twist.vel;
  const auto& trim = eom_.trimCondition();

  // TODO: 横系のトリムも考慮
  mpc_.current_state(eom_.kStateIdx_u) = linvel_B.x() - trim.u();
  mpc_.current_state(eom_.kStateIdx_alpha) = tobas::angleOfAttack(linvel_B) - trim.alpha();
  mpc_.current_state(eom_.kStateIdx_beta) = tobas::angleOfSideSlip(linvel_B);
  mpc_.current_state(eom_.kStateIdx_phi) = bs_ned_.pose.euler.roll;
  mpc_.current_state(eom_.kStateIdx_theta) = bs_ned_.pose.euler.pitch - trim.theta();
  mpc_.current_state(eom_.kStateIdx_p) = bs_ned_.twist.rot.x();
  mpc_.current_state(eom_.kStateIdx_q) = bs_ned_.twist.rot.y();
  mpc_.current_state(eom_.kStateIdx_r) = bs_ned_.twist.rot.z();

  // cout << mpc_.current_state << endl << endl;
}

void Controller::updateSetStateVector(double tar_roll, double tar_delta_pitch)
{
  mpc_.set_state(kCtrlIdx_u) = 0.;
  mpc_.set_state(kCtrlIdx_alpha) = 0.;
  mpc_.set_state(kCtrlIdx_beta) = 0.;
  mpc_.set_state(kCtrlIdx_phi) = tar_roll;
  mpc_.set_state(kCtrlIdx_theta) = tar_delta_pitch;
}

void Controller::updateRotorSpeeds(const VectorXd& thrust)
{
  ROS_ASSERT(thrust.rows() == x_rotors_.count());

  for (int i = 0; i < thrust.rows(); ++i)
  {
    if (thrust(i) < -1.)
    {
      dh_ros::rosFatal("Negative thrust force: " + to_string(thrust(i)) + " [N]");
      // TODO: 防御モードに移行
    }

    rotor_speeds_msg_.speeds[x_rotors_.rotorIdx(i)] =
      x_rotors_.thrustToRotSpeed(i, max(0., thrust(i)));
  }
}

void Controller::updateDeflections(const VectorXd& deflections)
{
  ROS_ASSERT(deflections.rows() == drone_.numControlSurfaces());

  deflections_msg_.deflections = eigen_tools::toStdVector(deflections);
}

void Controller::reconfigure(const ConfigType& cfg)
{
  ROS_ASSERT(cfg.prediction_horizon > 0.);
  ROS_ASSERT(cfg.prediction_steps > 0);
  ROS_ASSERT(cfg.beta_decay >= 0.);
  ROS_ASSERT(cfg.attitude_decay >= 0.);
  ROS_ASSERT(cfg.beta_weight > 0.);
  ROS_ASSERT(cfg.attitude_weight > 0.);

  mpc_.time_step = cfg.prediction_horizon / cfg.prediction_steps;
  mpc_.prediction_steps = mpc_.input_steps = cfg.prediction_steps;
  mpc_.decay_time_consts(kCtrlIdx_u) = cfg.forward_speed_decay;
  mpc_.decay_time_consts(kCtrlIdx_alpha) = cfg.alpha_decay;
  mpc_.decay_time_consts(kCtrlIdx_beta) = cfg.attitude_decay;
  mpc_.decay_time_consts(kCtrlIdx_phi) = mpc_.decay_time_consts(kCtrlIdx_theta) =
    cfg.attitude_decay;

  const ctrl::LinearDynamics cont(eom_.A(), eom_.B());
  const auto disc = c2d_->convert(cont, mpc_.time_step);
  mpc_.discrete_dynamics.resize(cfg.prediction_steps, disc);

  // 制御変数の重み
  mpc_.control_weight(kCtrlIdx_u) = cfg.forward_speed_weight;
  mpc_.control_weight(kCtrlIdx_alpha) = cfg.alpha_weight;
  mpc_.control_weight(kCtrlIdx_beta) = cfg.beta_weight;
  mpc_.control_weight(kCtrlIdx_phi) = mpc_.control_weight(kCtrlIdx_theta) = cfg.attitude_weight;

  // 制御入力の重み
  mpc_.input_weight.block(0, 0, x_rotors_.count(), 1) =
    VectorXd::Constant(x_rotors_.count(), pow(10, cfg.thrust_weight_exp));
  mpc_.input_weight.block(x_rotors_.count(), 0, drone_.numControlSurfaces(), 1) =
    VectorXd::Constant(drone_.numControlSurfaces(), pow(10, cfg.deflection_weight_exp));

  // 制御入力の変化率の重み
  mpc_.input_rate_weight.block(0, 0, x_rotors_.count(), 1) =
    VectorXd::Constant(x_rotors_.count(), pow(10, cfg.thrust_rate_weight_exp));
  mpc_.input_rate_weight.block(x_rotors_.count(), 0, drone_.numControlSurfaces(), 1) =
    VectorXd::Constant(drone_.numControlSurfaces(), pow(10, cfg.deflection_rate_weight_exp));
}

void Controller::airPressureCb(const sensor_msgs::FluidPressure& msg)
{
  if (!pressure_received_)
  {
    pressure_received_ = true;
  }

  air_density_ = dh_std::pressureToDensity(msg.fluid_pressure);
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
        dh_ros::rosInfo("Controller is ready.");
        check_topics_timer_.stop();
        state_ = TAKEOFF;
      }
      break;
    }
    case TAKEOFF:
    {
      publishTakeoffCommand();

      // 失速しない最低速度を上回ったら制御開始
      const auto cur_speed = bs_nwu.twist.vel.Norm();
      if (cur_speed > eom_.trimCondition().speedLimit(air_density_).lower)
      {
        setInitialTarget();
        state_ = FLIGHT;
        dh_ros::rosInfo("The aircraft takes off and begins flight control.");
      }
      break;
    }
    case FLIGHT:
    {
      runOnce();
      break;
    }
  }
}

void Controller::commandCb(const CmdMsg& cmd_nwu)
{
  if (!(state_ == FLIGHT))
  {
    dh_ros::rosError("Not in flight state.");
    return;
  }

  if (!eom_.trimCondition().speedLimit(air_density_).inRange(cmd_nwu.speed))
  {
    dh_ros::rosError("Invalid speed is commanded.");
    return;
  }

  tf::speedRollDeltaPitchNwuToNed(cmd_nwu, cmd_ned_);
}

void Controller::checkTopicsTimerCb(const ros::TimerEvent& event)
{
  if (!pressure_received_)
  {
    dh_ros::rosWarn("Air pressure is not received yet.");
  }

  if (!bs_received_)
  {
    dh_ros::rosWarn("Base state is not received yet.");
  }
}

void Controller::dynamicReconfigureCb(const ConfigType& cfg, uint32_t level)
{
  reconfigure(cfg);
}
}  // namespace tobas_fixed_wing_controller
