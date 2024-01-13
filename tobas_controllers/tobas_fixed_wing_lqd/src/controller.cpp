#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_kdl/frames.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>

#include <tobas_tools/conversions/coordinates.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Throttles.h>

#include "../include/tobas_fixed_wing_lqd/controller.hpp"
#include "../include/tobas_fixed_wing_lqd/constants.hpp"

using namespace std;
using namespace Eigen;
using namespace tobas_std;

namespace tobas_fixed_wing_lqd
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
    ROS_THROW_NAMED(name_, "The number of propellers is zero.");

  q_0_.resize(drone_.tree().getNrOfJoints());

  setScales();
  lqd_.state_weight.resize(eom_.kStateSize);
  lqd_.input_weight.resize(eom_.inputSize());
  lqd_.input_rate_weight.resize(eom_.inputSize());
  lqd_.current_state.resize(eom_.kStateSize);
  lqd_.target_state.resize(eom_.kStateSize);

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
  throttles_pub_ = nh_.advertise<tobas_msgs::Throttles>(tobas::kThrottlesCmdTopic, 1);
  deflections_pub_ =
    nh_.advertise<tobas_msgs::ControlSurfaceDeflections>(tobas::kDeflectionCmdTopic, 1);
  feedback_pub_ =
    nh_.advertise<tobas_msgs::FixedWingControllerFeedback>("fixed_wing_controller_feedback", 1);
}

void Controller::registerSubscribers()
{
  super::registerSubscribers();

  air_pressure_sub_ =
    nh_.subscribe(tobas::kAirPressureTopic, 1, &Controller::airPressureCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryTopic, 1, &Controller::batteryCb, this, tcpNoDelay());
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &Controller::odomCb, this, tcpNoDelay());
  cmd_sub_ =
    nh_.subscribe(tobas::kSpeedRollDpitchCmdTopic, 1, &Controller::commandCb, this, tcpNoDelay());
}

bool Controller::isReady()
{
  return pressure_received_ && battery_received_ && odom_received_;
}

void Controller::initialize()
{
  // 最新の制御時刻
  t_last_loop_ = odom_ned_.header.stamp;

  // 制御入力の初期値
  lqd_.last_input = VectorXd::Zero(eom_.inputSize());

  // コマンドの初期値
  const auto& trim = eom_.trimCondition();
  cmd_ned_.speed = trim.takeOffSpeed(rho_);
  cmd_ned_.roll = 0.;
  cmd_ned_.delta_pitch = kInitialDeltaPitch;
}

void Controller::runOnce()
{
  // 時刻を更新
  const auto& cur_time = odom_ned_.header.stamp;
  const auto dt = (cur_time - t_last_loop_).toSec();
  t_last_loop_ = cur_time;

  // 現在の速度を使って状態方程式を更新
  switch (eom_.update(odom_ned_.twist.vel.norm(), rho_, battery_->voltage, q_0_))
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

  lqd_.dynamics.A = eom_.A();
  lqd_.dynamics.B = eom_.B();

  updateCurrentStateVector();
  updateSetStateVector(cmd_ned_.roll, cmd_ned_.delta_pitch);

  // 最適制御入力を求める
  const VectorXd du = lqd_.solve(dt);
  const VectorXd u = eom_.trimInput() + du;

  // For debug
  // cout << "A_cont:" << endl << eom_.A() << endl;
  // cout << "B_cont:" << endl << eom_.B() << endl;
  // cout << lqd_ << endl;

  const VectorXd thrust = u.block(0, 0, x_rotors_.count(), 1);
  const VectorXd deflections = u.block(x_rotors_.count(), 0, drone_.numControlSurfaces(), 1);

  // Publish
  publishRotorSpeeds(thrust);
  publishDeflections(deflections);
  publishFeedback(du);
}

void Controller::setScales()
{
  // 状態変数のスケール
  lqd_.state_scale.resize(eom_.kStateSize);
  lqd_.state_scale(eom_.kStateIdx_u) = eom_.trimCondition().takeOffSpeed(kStandardAirDensity);
  lqd_.state_scale(eom_.kStateIdx_alpha) = drone_.vehicle().alpha_limit.range();
  lqd_.state_scale(eom_.kStateIdx_beta) = M_PI_4;
  lqd_.state_scale(eom_.kStateIdx_phi) = M_PI_4;
  lqd_.state_scale(eom_.kStateIdx_theta) = M_PI_4;
  lqd_.state_scale(eom_.kStateIdx_p) = M_PI;
  lqd_.state_scale(eom_.kStateIdx_q) = M_PI;
  lqd_.state_scale(eom_.kStateIdx_r) = M_PI;

  // 制御入力のスケール
  lqd_.input_scale.resize(eom_.inputSize());
  const auto thrust_scale = tobas::getMass() * tobas::kGravity / x_rotors_.count();
  lqd_.input_scale.block(0, 0, x_rotors_.count(), 1).fill(thrust_scale);
  for (size_t i = 0; i < drone_.numControlSurfaces(); ++i)
  {
    lqd_.input_scale(x_rotors_.count() + i) = drone_.controlSurface(i).angle_limit.range();
  }
}

void Controller::updateCurrentStateVector()
{
  const auto& trim = eom_.trimCondition();

  // TODO: 横系のトリムも考慮
  lqd_.current_state(eom_.kStateIdx_u) = odom_ned_.twist.vel.x() - trim.u();
  lqd_.current_state(eom_.kStateIdx_alpha) =
    tobas::angleOfAttack(odom_ned_.twist.vel) - trim.alpha();
  lqd_.current_state(eom_.kStateIdx_beta) = tobas::angleOfSideSlip(odom_ned_.twist.vel);
  lqd_.current_state(eom_.kStateIdx_phi) = odom_ned_.pose.euler.roll;
  lqd_.current_state(eom_.kStateIdx_theta) = odom_ned_.pose.euler.pitch - trim.theta();
  lqd_.current_state(eom_.kStateIdx_p) = odom_ned_.twist.rot.x();
  lqd_.current_state(eom_.kStateIdx_q) = odom_ned_.twist.rot.y();
  lqd_.current_state(eom_.kStateIdx_r) = odom_ned_.twist.rot.z();
}

void Controller::updateSetStateVector(const double& tar_roll, const double& tar_delta_pitch)
{
  const auto& trim = eom_.trimCondition();
  const auto tar_u = cmd_ned_.speed * cos(eom_.trimCondition().alpha());

  lqd_.target_state(eom_.kStateIdx_u) = tar_u - trim.u();
  lqd_.target_state(eom_.kStateIdx_alpha) = 0.;
  lqd_.target_state(eom_.kStateIdx_beta) = 0.;
  lqd_.target_state(eom_.kStateIdx_phi) = tar_roll;
  lqd_.target_state(eom_.kStateIdx_theta) = tar_delta_pitch;
  lqd_.target_state(eom_.kStateIdx_p) = 0.;
  lqd_.target_state(eom_.kStateIdx_q) = 0.;
  lqd_.target_state(eom_.kStateIdx_r) = 0.;
}

void Controller::publishRotorSpeeds(const Eigen::VectorXd& thrust)
{
  const auto throttles_msg = boost::make_shared<tobas_msgs::Throttles>();
  throttles_msg->header.stamp = odom_ned_.header.stamp;

  throttles_msg->data.resize(drone_.numRotors(), tobas::kArmThrottle);
  for (int i = 0; i < thrust.rows(); ++i)
  {
    throttles_msg->data[x_rotors_.rotorIdx(i)] =
      x_rotors_.throttleFromThrust(i, max(0., thrust(i)), battery_->voltage);
  }

  throttles_pub_.publish(throttles_msg);
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

void Controller::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      check_topics_timer_.stop();
      break;
    default:
      break;
  }
}

void Controller::airPressureCb(const sensor_msgs::FluidPressureConstPtr& msg)
{
  rho_ = tobas_std::pressureToDensity(msg->fluid_pressure);

  if (!pressure_received_)
    pressure_received_ = true;
}

void Controller::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;

  if (!battery_received_)
    battery_received_ = true;
}

void Controller::odomCb(const tobas_msgs::OdometryConstPtr& odom_nwu)
{
  // コールバックの時点で全てNED座標系に変換しておく
  tf::baseStateNwuToNed(*odom_nwu, odom_ned_);

  if (!odom_received_)
    odom_received_ = true;

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

  runOnce();
}

void Controller::commandCb(const tobas_msgs::SpeedRollDeltaPitchConstPtr& cmd_nwu)
{
  if (!(state_ == FLIGHT))
  {
    rosError(name_, "Not in flight state.");
    return;
  }

  if (!eom_.trimCondition().speedLimit(rho_).inRange(cmd_nwu->speed))
  {
    rosError(name_, "Invalid speed is commanded.");
    return;
  }

  tf::speedRollDeltaPitchNwuToNed(*cmd_nwu, cmd_ned_);
}

void Controller::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!pressure_received_)
    rosInfo(name_, "Waiting for " << ns() << tobas::kAirPressureTopic);

  if (!battery_received_)
    rosInfo(name_, "Waiting for " << ns() << tobas::kBatteryTopic);

  if (!odom_received_)
    rosInfo(name_, "Waiting for " << ns() << tobas::kOdometryTopic);
}

void Controller::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  ROS_ASSERT(cfg.forward_speed_weight > 0.);
  ROS_ASSERT(cfg.alpha_weight > 0.);
  ROS_ASSERT(cfg.beta_weight > 0.);
  ROS_ASSERT(cfg.attitude_weight > 0.);
  ROS_ASSERT(cfg.angular_velocity_weight > 0.);

  // 状態変数の重み
  lqd_.state_weight(eom_.kStateIdx_u) = cfg.forward_speed_weight;
  lqd_.state_weight(eom_.kStateIdx_alpha) = cfg.alpha_weight;
  lqd_.state_weight(eom_.kStateIdx_beta) = cfg.beta_weight;
  lqd_.state_weight(eom_.kStateIdx_phi) = cfg.attitude_weight;
  lqd_.state_weight(eom_.kStateIdx_theta) = cfg.attitude_weight;
  lqd_.state_weight(eom_.kStateIdx_p) = cfg.angular_velocity_weight;
  lqd_.state_weight(eom_.kStateIdx_q) = cfg.angular_velocity_weight;
  lqd_.state_weight(eom_.kStateIdx_r) = cfg.angular_velocity_weight;

  // 制御入力の重み
  const auto thrust_weight = exp10(cfg.thrust_weight_log10);
  const auto deflection_weight = exp10(cfg.deflection_weight_log10);
  lqd_.input_weight.head(x_rotors_.count()).fill(thrust_weight);
  lqd_.input_weight.tail(drone_.numControlSurfaces()).fill(deflection_weight);

  // 制御入力の変化率の重み
  const auto thrust_rate_weight = exp10(cfg.thrust_rate_weight_log10);
  const auto deflection_rate_weight = exp10(cfg.deflection_rate_weight_log10);
  lqd_.input_rate_weight.head(x_rotors_.count()).fill(thrust_rate_weight);
  lqd_.input_rate_weight.tail(drone_.numControlSurfaces()).fill(deflection_rate_weight);
}
}  // namespace tobas_fixed_wing_lqd
