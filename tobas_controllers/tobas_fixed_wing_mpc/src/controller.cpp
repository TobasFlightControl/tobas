#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_kdl/frames.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_linear_control/util.hpp>

#include <tobas_tools/conversions/coordinates.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/RotorSpeeds.h>

#include "../include/tobas_fixed_wing_mpc/controller.hpp"
#include "../include/tobas_fixed_wing_mpc/constants.hpp"

using namespace std;
using namespace Eigen;
using namespace tobas_std;

namespace tobas_fixed_wing_mpc
{
Controller::Controller(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name),
    x_rotors_(drone_, tobas::Axis::X_POSITIVE),
    eom_(drone_),
    check_topics_timer_(nh_, tobas::kCheckTopicsTimerPeriod, &Controller::checkTopicsTimerCb, this),
    server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  x_rotors_.updateInternalDataStructures();
  eom_.updateInternalDataStructures();

  if (x_rotors_.count() == 0)
    ROS_EXIT_NAMED(nh_, name_, "The number of propellers is zero.");

  q_0_.resize(drone_.tree().getNrOfJoints());
  c2d_.resize(eom_.kStateSize, eom_.inputSize());

  mpc_.decay_time_consts.resize(kCtrlSize);
  setCz();
  setScales();
  mpc_.input_rate_weight.resize(eom_.inputSize());
  mpc_.input_weight.resize(eom_.inputSize());
  mpc_.control_weight.resize(kCtrlSize);
  mpc_.current_state.resize(eom_.kStateSize);
  mpc_.set_state.resize(kCtrlSize);

  registerPublishers();
  registerSubscribers();

  // Dynamic Reconfigure
  ConfigServer::CallbackType f = boost::bind(&Controller::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void Controller::getRosParams()
{
}

void Controller::registerPublishers()
{
  rot_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>(tobas::kRotorSpeedsCmdTopic, 1);
  deflections_pub_ =
    nh_.advertise<tobas_msgs::ControlSurfaceDeflections>(tobas::kDeflectionCmdTopic, 1);
  feedback_pub_ =
    nh_.advertise<tobas_msgs::FixedWingControllerFeedback>("fixed_wing_controller_feedback", 1);
}

void Controller::registerSubscribers()
{
  air_pressure_sub_ =
    nh_.subscribe(tobas::kAirPressureTopic, 1, &Controller::airPressureCb, this, tcpNoDelay());
  battery_sub_ =
    nh_.subscribe(tobas::kBatteryLpfTopic, 1, &Controller::batteryCb, this, tcpNoDelay());
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &Controller::odomCb, this, tcpNoDelay());
  cmd_sub_ =
    nh_.subscribe(tobas::kSpeedRollDpitchCmdTopic, 1, &Controller::commandCb, this, tcpNoDelay());
}

bool Controller::isReady()
{
  return air_pressure_ != nullptr && battery_ != nullptr && odom_nwu_ != nullptr;
}

void Controller::initialize()
{
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
  for (size_t i = 0; i < drone_.numControlSurfaces(); ++i)
  {
    mpc_.input_scale(x_rotors_.count() + i) = drone_.controlSurface(i).angle_limit.range();
  }
}

void Controller::setInputConstraint()
{
  const VectorXd lb = eom_.minDeltaInput();
  const VectorXd ub = eom_.maxDeltaInput();
  tobas_std::fill(mpc_.input_ineqs, ctrl::matIneqFromRange(lb, ub));
}

void Controller::setInputRateConstraint()
{
  VectorXd lb = VectorXd::Constant(eom_.inputSize(), numeric_limits<double>::lowest());
  VectorXd ub = VectorXd::Constant(eom_.inputSize(), numeric_limits<double>::max());

  // FIXME: 遅延が大きいなら舵角の変化率の制約は消してもいいかも
  for (size_t i = 0; i < drone_.numControlSurfaces(); ++i)
  {
    const auto& max_angle_rate = drone_.controlSurface(i).max_angle_rate;
    lb(x_rotors_.count() + i) = -max_angle_rate;
    ub(x_rotors_.count() + i) = +max_angle_rate;
  }

  tobas_std::fill(mpc_.input_rate_ineqs, ctrl::matIneqFromRange(lb, ub));
}

void Controller::updateCurrentStateVector()
{
  const auto& trim = eom_.trimCondition();
  odom_ned_.frame.M.getRPY(cur_roll_, cur_pitch_, cur_yaw_);

  // TODO: 横系のトリムも考慮
  mpc_.current_state(eom_.kStateIdx_u) = odom_ned_.twist.vel.x() - trim.u();
  mpc_.current_state(eom_.kStateIdx_alpha) =
    tobas::angleOfAttack(odom_ned_.twist.vel) - trim.alpha();
  mpc_.current_state(eom_.kStateIdx_beta) = tobas::angleOfSideSlip(odom_ned_.twist.vel);
  mpc_.current_state(eom_.kStateIdx_phi) = cur_roll_;
  mpc_.current_state(eom_.kStateIdx_theta) = cur_pitch_ - trim.theta();
  mpc_.current_state(eom_.kStateIdx_p) = odom_ned_.twist.rot.x();
  mpc_.current_state(eom_.kStateIdx_q) = odom_ned_.twist.rot.y();
  mpc_.current_state(eom_.kStateIdx_r) = odom_ned_.twist.rot.z();
}

void Controller::updateSetStateVector()
{
  const auto& trim = eom_.trimCondition();

  // 失速しないように速度制限をした上で目標推力を計算
  const auto rho = tobas_std::pressureToDensity(air_pressure_->fluid_pressure);
  const auto tar_speed = trim.speedLimit(rho).clamp(cmd_ned_.speed);
  const auto tar_u = tar_speed * cos(eom_.trimCondition().alpha());

  mpc_.set_state(kCtrlIdx_u) = tar_u - trim.u();
  mpc_.set_state(kCtrlIdx_alpha) = 0.;
  mpc_.set_state(kCtrlIdx_beta) = 0.;
  mpc_.set_state(kCtrlIdx_phi) = cmd_ned_.roll;
  mpc_.set_state(kCtrlIdx_theta) = cmd_ned_.delta_pitch;
  mpc_.set_state(kCtrlIdx_p) = 0.;
  mpc_.set_state(kCtrlIdx_q) = 0.;
  mpc_.set_state(kCtrlIdx_r) = 0.;
}

void Controller::publishRotSpeeds(const Eigen::VectorXd& thrust)
{
  const auto rot_speeds = boost::make_shared<tobas_msgs::RotorSpeeds>();
  rot_speeds->header.stamp = odom_ned_.header.stamp;

  rot_speeds->speeds.resize(drone_.numRotors(), 0.);
  for (size_t i = 0; i < static_cast<size_t>(thrust.rows()); ++i)
    rot_speeds->speeds[x_rotors_.rotorIdx(i)] = x_rotors_.rotSpeedFromThrust(i, max(0., thrust(i)));

  rot_speeds_pub_.publish(rot_speeds);
}

void Controller::publishDeflections(const Eigen::VectorXd& deflections)
{
  const auto deflections_msg = boost::make_shared<tobas_msgs::ControlSurfaceDeflections>();
  deflections_msg->header.stamp = odom_ned_.header.stamp;
  deflections_msg->deflections = eigen_tools::toStdVector(deflections);
  deflections_pub_.publish(deflections_msg);
}

void Controller::publishFeedback(const Eigen::VectorXd& du)
{
  const auto& trim = eom_.trimCondition();
  const auto feedback = boost::make_shared<tobas_msgs::FixedWingControllerFeedback>();

  feedback->trim_thrusts.resize(drone_.numRotors());
  feedback->delta_thrusts.resize(drone_.numRotors());
  feedback->trim_deflections.resize(drone_.numControlSurfaces());
  feedback->delta_deflections.resize(drone_.numControlSurfaces());

  feedback->trim_u = trim.u();
  feedback->trim_alpha = trim.alpha();

  for (size_t i = 0; i < x_rotors_.count(); ++i)
  {
    feedback->trim_thrusts[x_rotors_.rotorIdx(i)] = eom_.trimInput()(i);
    feedback->delta_thrusts[x_rotors_.rotorIdx(i)] = du(i);
  }

  for (size_t i = 0; i < drone_.numControlSurfaces(); ++i)
  {
    const auto u_idx = x_rotors_.count() + i;
    feedback->trim_deflections[i] = eom_.trimInput()[u_idx];
    feedback->delta_deflections[i] = du(u_idx);
  }

  feedback_pub_.publish(feedback);
}

void Controller::airPressureCb(const sensor_msgs::FluidPressureConstPtr& msg)
{
  air_pressure_ = msg;
}

void Controller::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void Controller::odomCb(const tobas_msgs::OdometryConstPtr& odom_nwu)
{
  if (odom_nwu->status != tobas_msgs::Odometry::NO_ERROR)
    return;

  odom_nwu_ = odom_nwu;

  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      initialize();
      is_initialized_ = true;
      TOBAS_GOOD("Controller is ready.");
    }
    return;
  }

  // コマンドが来ていなければスキップ
  if (cmd_nwu_ == nullptr)
    return;

  // NWU -> NED
  tf::odometryNwuToNed(*odom_nwu_, odom_ned_);
  tf::speedRollDeltaPitchNwuToNed(*cmd_nwu_, cmd_ned_);

  // 状態方程式を更新
  const auto rho = tobas_std::pressureToDensity(air_pressure_->fluid_pressure);
  switch (eom_.update(cmd_ned_.speed, rho, battery_->voltage, q_0_))
  {
    case tobas::SolverI::E_NO_ERROR:
      break;
    case tobas::SolverI::E_WARN:
      rosWarn(name_, eom_.errorMessage());
      break;
    case tobas::SolverI::E_ERROR:
      rosError(name_, eom_.errorMessage());
      return;
    default:
      rosWarn(name_, "Unknown error code from MicroDisturbanceEoM.");
      break;
  }
  const ctrl::LinearDynamics cont(eom_.A(), eom_.B());
  const auto disc = c2d_.convert(cont, mpc_.time_step);
  fill(mpc_.discrete_dynamics, disc);

  setInputConstraint();  // EoMの更新後に呼ぶ必要がある
  updateCurrentStateVector();
  updateSetStateVector();

  // MPCを解いて最適制御入力を求める
  mpc_.solve();
  const VectorXd& du = mpc_.optimalControlInput();
  const VectorXd u = eom_.trimInput() + du;

  // For debug
  // cout << "A_cont:" << endl << eom_.A() << endl;
  // cout << "B_cont:" << endl << eom_.B() << endl;
  // cout << "Discrete dynamics:" << endl << disc << endl;
  // cout << mpc_ << endl;

  // For debug
  // cout << "A_cont:" << endl << eom_.A() << endl;
  // cout << "B_cont:" << endl << eom_.B() << endl;
  // cout << lqd_ << endl;

  const VectorXd thrust = u.block(0, 0, x_rotors_.count(), 1);
  const VectorXd deflections = u.block(x_rotors_.count(), 0, drone_.numControlSurfaces(), 1);

  // Publish
  publishRotSpeeds(thrust);
  publishDeflections(deflections);
  publishFeedback(du);
}

void Controller::commandCb(const tobas_msgs::SpeedRollDeltaPitchConstPtr& cmd_nwu)
{
  if (!is_initialized_)
    return;

  tf::speedRollDeltaPitchNwuToNed(*cmd_nwu, cmd_ned_);
}

void Controller::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (air_pressure_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kAirPressureTopic);

  if (battery_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kBatteryLpfTopic);

  if (odom_nwu_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kOdometryTopic);
}

void Controller::dynamicReconfigureCb(const ConfigType& cfg, size_t)
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
  const auto disc = c2d_.convert(cont, mpc_.time_step);
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
  mpc_.input_weight.head(x_rotors_.count()).fill(exp10(cfg.thrust_weight_log10));
  mpc_.input_weight.tail(drone_.numControlSurfaces()).fill(exp10(cfg.deflection_weight_log10));

  // 制御入力の変化率の重み
  mpc_.input_rate_weight.head(x_rotors_.count()).fill(exp10(cfg.thrust_rate_weight_log10));
  mpc_.input_rate_weight.tail(drone_.numControlSurfaces())
    .fill(exp10(cfg.deflection_rate_weight_log10));

  mpc_.input_rate_eqs.resize(cfg.prediction_steps, ctrl::LinearEquation(eom_.inputSize(), 0));
  mpc_.input_eqs.resize(cfg.prediction_steps, ctrl::LinearEquation(eom_.inputSize(), 0));
  mpc_.control_eqs.resize(cfg.prediction_steps, ctrl::LinearEquation(kCtrlSize, 0));
  mpc_.input_rate_ineqs.resize(cfg.prediction_steps);
  mpc_.input_ineqs.resize(cfg.prediction_steps);
  mpc_.control_ineqs.resize(cfg.prediction_steps, ctrl::LinearEquation(kCtrlSize, 0));

  setInputRateConstraint();
}
}  // namespace tobas_fixed_wing_mpc
